"""Instruction-level simulator for the 80C552 car wash firmware.

The simulator reads the A51 source file directly (no assembler needed) and executes the
subset of the 8051 instruction set that the firmware uses. It models:

  * internal RAM, bit-addressable area, register bank 0 and the stack
  * Timer 0 in mode 1 (16-bit) with its overflow interrupt (vector 0x0B)
  * the 80C552 ADC: conversion start through ADCON.ADCS, result in ADCH,
    end-of-conversion interrupt (vector 0x53)
  * the PWM0 / PWM1 / PWMP registers (values are recorded, not modulated)
  * I/O ports: input pins are read from the plant model, outputs from the port latch

Timing: one machine cycle = 1 us (12 MHz crystal). Instruction cycle counts follow the
8051 data sheet (1 or 2 cycles), which is what matters for the 50 ms timer interrupt.
"""
import re

# ---------------------------------------------------------------- SFR map
SFR = {'ACC': 0xE0, 'PSW': 0xD0, 'SP': 0x81, 'TCON': 0x88, 'TMOD': 0x89, 'TL0': 0x8A,
       'TH0': 0x8C, 'P0': 0x80, 'P1': 0x90, 'P2': 0xA0}
SFR_BITS = {'TR0': 0x8C, 'TF0': 0x8D}
ACC, PSW, SP, TCON, TL0, TH0 = 0xE0, 0xD0, 0x81, 0x88, 0x8A, 0x8C
IEN0, ADCON, ADCH = 0xA8, 0xC5, 0xC6
ADCI, ADCS = 0x10, 0x08
TR0_MASK, TF0_MASK = 0x10, 0x20
ADC_CONVERSION_CYCLES = 50
TWO_CYCLE = {'AJMP', 'LJMP', 'SJMP', 'JMP', 'ACALL', 'LCALL', 'RET', 'RETI', 'PUSH',
                    'POP', 'JB', 'JNB', 'JC', 'JNC', 'JZ', 'JNZ', 'CJNE'}


class AsmError(Exception):
    pass


def parse_number(tok):
    t = tok.strip().upper()
    if t.startswith('0X'):
        return int(t[2:], 16)
    if re.fullmatch(r'[0-9][0-9A-F]*H', t):
        return int(t[:-1], 16)
    if t.isdigit():
        return int(t)
    return None


class Program:
    """Parsed A51 source: instructions, labels, EQU symbols and ORG vectors."""

    def __init__(self, path):
        self.path = path
        self.equ, self.labels, self.ins, self.vectors = {}, {}, [], {}
        with open(path, newline='') as f:
            lines = f.read().splitlines()
        for lineno, raw in enumerate(lines, 1):
            code = raw.split(';', 1)[0].strip()
            if not code:
                continue
            m = re.match(r'^([A-Za-z_]\w*):\s*(.*)$', code)
            if m:
                self.labels[m.group(1).upper()] = len(self.ins)
                code = m.group(2).strip()
                if not code:
                    continue
            parts = code.split(None, 1)
            if len(parts) > 1 and parts[1].split()[0].upper() == 'EQU':
                self.equ[parts[0].upper()] = parts[1].split(None, 1)[1].strip().upper()
                continue
            op = parts[0].upper()
            if op == 'ORG':
                self.vectors[parse_number(parts[1])] = len(self.ins)
                continue
            if op == 'END':
                break
            args = [a.strip().upper() for a in parts[1].split(',')] if len(parts) > 1 else []
            self.ins.append((op, args, lineno, raw.strip()))

    def resolve(self, tok):
        for _ in range(10):
            if tok not in self.equ:
                return tok
            tok = self.equ[tok]
        raise AsmError('EQU loop at ' + tok)


class CPU:
    def __init__(self, program, plant):
        self.p = program
        self.plant = plant
        self.ram = [0] * 256
        self.ram[SP] = 0x07
        self.ram[0x80] = self.ram[0x90] = self.ram[0xA0] = 0xFF
        self.cycles = 0
        self.pc = program.vectors[0x00]
        self.dptr = 0
        self.in_isr = False
        self.adc_done_at = None
        self.t0_running = False
        self.t0_base_cycles = 0
        self.t0_base_value = 0
        self.t0_overflow_at = None
        self.code = [self._decode(i) for i in range(len(program.ins))]

    # ------------------------------------------------------------ operand helpers
    def _byte_addr(self, tok):
        tok = self.p.resolve(tok)
        if tok in SFR:
            return SFR[tok]
        n = parse_number(tok)
        if n is None:
            raise AsmError('not a byte address: ' + tok)
        return n

    def _bit_addr(self, tok):
        tok = self.p.resolve(tok)
        if tok in SFR_BITS:
            return SFR_BITS[tok]
        if '.' in tok:
            byte, bit = tok.split('.')
            a = self._byte_addr(byte)
            return (a - 0x20) * 8 + int(bit) if a < 0x80 else a + int(bit)
        n = parse_number(tok)
        if n is None:
            raise AsmError('not a bit address: ' + tok)
        return n          # a plain number in a bit context is a bit address (A51 rule)

    def _reg(self, tok):
        tok = self.p.resolve(tok)
        return int(tok[1]) if re.fullmatch(r'R[0-7]', tok) else None

    def _reader(self, tok):
        """Return a function that reads the value of a source operand."""
        ram = self.ram
        if tok.startswith('#'):
            v = parse_number(tok[1:])
            if v is None:
                v = self.p.labels[tok[1:]]      # #label (used for DPTR tables)
            return lambda: v
        if tok == 'A':
            return lambda: ram[ACC]
        r = self._reg(tok)
        a = r if r is not None else self._byte_addr(tok)
        return lambda: ram[a]

    def _writer(self, tok):
        r = self._reg(tok)
        a = r if r is not None else (ACC if tok == 'A' else self._byte_addr(tok))
        return lambda v: self.write_byte(a, v)

    # ------------------------------------------------------------ memory
    def read_bit(self, b):
        if b in self.plant.input_pins:
            return self.plant.read_pin(b)
        if b < 0x80:
            return (self.ram[0x20 + (b >> 3)] >> (b & 7)) & 1
        return (self.ram[b & 0xF8] >> (b & 7)) & 1

    def write_bit(self, b, v):
        a, k = (0x20 + (b >> 3), b & 7) if b < 0x80 else (b & 0xF8, b & 7)
        self.write_byte(a, (self.ram[a] & ~(1 << k)) | (v << k))

    def write_byte(self, a, v):
        v &= 0xFF
        if a in (TL0, TH0, TCON):
            self._timer_sync()
            old = self.ram[a]
            self.ram[a] = v
            if a == TCON and (old ^ v) & TR0_MASK:
                self.t0_running = bool(v & TR0_MASK)
            self._timer_rebase()
            return
        if a == ADCON and (v & ADCS) and not (self.ram[a] & ADCS):
            self.adc_done_at = self.cycles + ADC_CONVERSION_CYCLES
        self.ram[a] = v

    # ------------------------------------------------------------ Timer 0 (mode 1)
    def _timer_sync(self):
        if self.t0_running:
            v = (self.t0_base_value + self.cycles - self.t0_base_cycles) & 0xFFFF
            self.ram[TH0], self.ram[TL0] = v >> 8, v & 0xFF

    def _timer_rebase(self):
        self.t0_base_cycles = self.cycles
        self.t0_base_value = self.ram[TH0] << 8 | self.ram[TL0]
        self.t0_overflow_at = (self.cycles + 0x10000 - self.t0_base_value) if self.t0_running else None

    # ------------------------------------------------------------ decoder
    def _decode(self, i):
        op, a, lineno, raw = self.p.ins[i]
        L = self.p.labels
        ram = self.ram
        nxt = i + 1

        def label(t):
            if t not in L:
                raise AsmError(f'line {lineno}: unknown label {t}')
            return L[t]

        def setC(c):
            ram[PSW] = (ram[PSW] & 0x7F) | (c << 7)

        if op == 'MOV':
            if a[0] == 'DPTR':
                rd = self._reader(a[1])
                def f():
                    self.dptr = rd(); return nxt
            else:
                rd, wr = self._reader(a[1]), self._writer(a[0])
                def f():
                    wr(rd()); return nxt
        elif op in ('ANL', 'ORL'):
            r0, r1, wr = self._reader(a[0]), self._reader(a[1]), self._writer(a[0])
            if op == 'ANL':
                def f():
                    wr(r0() & r1()); return nxt
            else:
                def f():
                    wr(r0() | r1()); return nxt
        elif op == 'INC':
            rd, wr = self._reader(a[0]), self._writer(a[0])
            def f():
                wr(rd() + 1); return nxt
        elif op == 'SUBB':
            rd = self._reader(a[1])
            def f():
                r = ram[ACC] - rd() - (ram[PSW] >> 7)
                setC(1 if r < 0 else 0); ram[ACC] = r & 0xFF; return nxt
        elif op == 'RL':
            def f():
                v = ram[ACC]; ram[ACC] = ((v << 1) | (v >> 7)) & 0xFF; return nxt
        elif op in ('SETB', 'CLR', 'CPL'):
            if a[0] == 'C':
                def f():
                    setC({'SETB': 1, 'CLR': 0}.get(op, 1 - (ram[PSW] >> 7))); return nxt
            elif a[0] == 'A':
                def f():
                    ram[ACC] = 0 if op == 'CLR' else ram[ACC] ^ 0xFF; return nxt
            else:
                b = self._bit_addr(a[0])
                if op == 'SETB':
                    def f():
                        self.write_bit(b, 1); return nxt
                elif op == 'CLR':
                    def f():
                        self.write_bit(b, 0); return nxt
                else:
                    def f():
                        self.write_bit(b, 1 - self.read_bit(b)); return nxt
        elif op in ('JB', 'JNB'):
            b, t, want = self._bit_addr(a[0]), label(a[1]), op == 'JB'
            def f():
                return t if bool(self.read_bit(b)) == want else nxt
        elif op in ('JC', 'JNC'):
            t, want = label(a[0]), 1 if op == 'JC' else 0
            def f():
                return t if ram[PSW] >> 7 == want else nxt
        elif op in ('JZ', 'JNZ'):
            t, want = label(a[0]), op == 'JZ'
            def f():
                return t if (ram[ACC] == 0) == want else nxt
        elif op == 'CJNE':
            r0, r1, t = self._reader(a[0]), self._reader(a[1]), label(a[2])
            def f():
                x, y = r0(), r1()
                setC(1 if x < y else 0)
                return t if x != y else nxt
        elif op in ('AJMP', 'LJMP', 'SJMP', 'JMP'):
            if a[0] == '@A+DPTR':
                def f():
                    # every jump table holds 2-byte AJMPs, so index = A / 2
                    j = self.dptr + ram[ACC] // 2
                    if ram[ACC] & 1 or j >= len(self.code) or self.p.ins[j][0] != 'AJMP':
                        raise AsmError(f'line {lineno}: jump table overrun (A={ram[ACC]})')
                    return j
            else:
                t = label(a[0])
                def f():
                    return t
        elif op in ('ACALL', 'LCALL'):
            t = label(a[0])
            def f():
                self.push(nxt & 0xFF); self.push(nxt >> 8); return t
        elif op in ('RET', 'RETI'):
            def f():
                hi = self.pop(); lo = self.pop()
                if op == 'RETI':
                    self.in_isr = False
                return hi << 8 | lo
        elif op == 'PUSH':
            ad = self._byte_addr(a[0])
            def f():
                self.push(ram[ad]); return nxt
        elif op == 'POP':
            ad = self._byte_addr(a[0])
            def f():
                ram[ad] = self.pop(); return nxt
        elif op == 'NOP':
            def f():
                return nxt
        else:
            raise AsmError(f'line {lineno}: instruction not supported by the simulator: {raw}')

        cyc = 2 if op in TWO_CYCLE else 1
        if op in ('MOV', 'ANL', 'ORL') and a[0] not in ('A', 'DPTR') and self._reg(a[0]) is None:
            cyc = 1 if a[1] == 'A' else 2          # direct,#data / direct,direct
        elif op == 'MOV' and a[0] == 'DPTR':
            cyc = 2
        return f, cyc

    # ------------------------------------------------------------ stack
    def push(self, v):
        self.ram[SP] = (self.ram[SP] + 1) & 0xFF
        self.ram[self.ram[SP]] = v & 0xFF

    def pop(self):
        v = self.ram[self.ram[SP]]
        self.ram[SP] = (self.ram[SP] - 1) & 0xFF
        return v

    # ------------------------------------------------------------ run
    def run(self, seconds, stop=None, plant_period=1000):
        """Run until `seconds` of simulated time or until stop() returns True."""
        end = int(seconds * 1e6)
        next_plant = self.cycles
        code, ram = self.code, self.ram
        while self.cycles < end:
            f, cyc = code[self.pc]
            self.pc = f()
            self.cycles += cyc
            c = self.cycles
            # Timer 0 overflow
            if self.t0_overflow_at is not None and c >= self.t0_overflow_at:
                ram[TCON] |= TF0_MASK
                self.t0_base_cycles, self.t0_base_value = self.t0_overflow_at, 0
                self.t0_overflow_at += 0x10000
            # ADC end of conversion
            if self.adc_done_at is not None and c >= self.adc_done_at:
                self.adc_done_at = None
                ram[ADCH] = self.plant.adc_read(ram[ADCON] & 0x07)
                ram[ADCON] = (ram[ADCON] | ADCI) & ~ADCS & 0xFF
            # interrupts (no nesting: the firmware uses a single priority level)
            if not self.in_isr and ram[IEN0] & 0x80:
                if ram[IEN0] & 0x02 and ram[TCON] & TF0_MASK:
                    ram[TCON] &= ~TF0_MASK & 0xFF
                    self._interrupt(0x0B)
                elif ram[IEN0] & 0x40 and ram[ADCON] & ADCI:
                    self._interrupt(0x53)
            if c >= next_plant:
                next_plant = c + plant_period
                self.plant.update(self)
                if stop and stop():
                    break

    def _interrupt(self, vector):
        self.in_isr = True
        self.push(self.pc & 0xFF)
        self.push(self.pc >> 8)
        self.pc = self.p.vectors[vector]
        self.cycles += 2

    # ------------------------------------------------------------ inspection
    def symbol_bit(self, name):
        return self.read_bit(self._bit_addr(name.upper()))

    def symbol_byte(self, name):
        r = self._reg(name.upper())
        return self.ram[r] if r is not None else self.ram[self._byte_addr(name.upper())]
