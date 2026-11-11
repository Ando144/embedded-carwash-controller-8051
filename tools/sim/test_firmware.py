"""Automated tests: run the real firmware in the simulator and check its behaviour.

    python -m unittest discover -s tools/sim -v
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plant import Car                       # noqa: E402
from run_sim import simulate                # noqa: E402

FULL_CYCLE = list(range(21)) + [0]          # E0 -> E1 -> ... -> E20 -> E0
ACTUATORS = ['BRIDGE_FWD', 'BRIDGE_BACK', 'VR_IN', 'VR_OUT', 'HR_UP', 'HR_DOWN',
             'VALVE_WATER', 'VALVE_SOAP', 'ALARM_START', 'LED_NORMAL', 'LED_INTENSE']


def turned_on(wash):
    """Outputs switched on at least once after Init (port latches are 1 at reset)."""
    init_done = wash.transitions[0][0] + 0.01
    return {n for t, n, v in wash.events if v == 1 and t > init_done}


def pwm_during(wash, state):
    return {(s.pwm0, s.pwm1) for s in wash.samples if s.state == state}


class NormalProgram(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.wash, cls.cpu = simulate([Car(intense=False)])

    def test_runs_the_full_cycle_in_order(self):
        self.assertEqual(self.wash.state_sequence(), FULL_CYCLE)

    def test_4_second_delays(self):
        for state in (5, 11, 16):
            with self.subTest(state=f'E{state}'):
                self.assertAlmostEqual(self.wash.time_in_state(state), 4.0, delta=0.06)

    def test_rollers_follow_the_car_profile(self):
        brushing = [s for s in self.wash.samples if s.state == 9 and 0 <= s.x <= 450]
        self.assertGreater(len(brushing), 50)
        for s in brushing:
            self.assertTrue(28 <= s.h_dist <= 52, f'side distance {s.h_dist:.1f} cm at t={s.t:.2f}')
            self.assertTrue(28 <= s.v_dist <= 52, f'roof distance {s.v_dist:.1f} cm at t={s.t:.2f}')

    def test_pwm_levels(self):
        self.assertEqual(pwm_during(self.wash, 9), {(0x80, 0xFF)})    # brush 50 %, dryer off
        self.assertEqual(pwm_during(self.wash, 18), {(0xFF, 0x80)})   # brush off, dryer 50 %
        self.assertEqual(self.cpu.symbol_byte('PWM0'), 0xFF)
        self.assertEqual(self.cpu.symbol_byte('PWM1'), 0xFF)

    def test_pwm_prescaler_gives_about_1_khz(self):
        pwmp = self.cpu.symbol_byte('PWMP')
        self.assertEqual(pwmp, 0x17)
        self.assertAlmostEqual(12e6 / (2 * 255 * (1 + pwmp)), 1000, delta=25)

    def test_correction_flags_are_clear_after_the_car(self):
        for flag in ('VR_IN_ON', 'VR_OUT_ON', 'HR_UP_ON', 'HR_DOWN_ON', 'bridge_reversed'):
            with self.subTest(flag=flag):
                self.assertEqual(self.cpu.symbol_bit(flag), 0)

    def test_program_led(self):
        self.assertIn('LED_NORMAL', turned_on(self.wash))
        self.assertNotIn('LED_INTENSE', turned_on(self.wash))

    def test_green_light_blinks_at_1_hz(self):
        e20 = next(t for t, s in self.wash.transitions if s == 20)
        toggles = [t for t in self.wash.edges('LIGHT_GREEN') if t > e20 + 0.01]
        gaps = [b - a for a, b in zip(toggles, toggles[1:])]
        self.assertTrue(gaps)
        for g in gaps:
            self.assertAlmostEqual(g, 1.0, delta=0.06)

    def test_back_to_idle_with_everything_off(self):
        self.assertEqual(self.wash.state, 0)
        self.assertEqual(self.cpu.symbol_bit('TR0'), 0, 'Timer 0 must be stopped in Idle')
        self.assertEqual(self.cpu.symbol_bit('LIGHT_GREEN'), 1)
        self.assertEqual(self.cpu.symbol_bit('LIGHT_RED'), 0)
        for name in ACTUATORS:
            with self.subTest(output=name):
                self.assertEqual(self.cpu.symbol_bit(name), 0)


class IntenseProgram(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.wash, cls.cpu = simulate([Car(intense=True)])

    def test_runs_the_full_cycle_in_order(self):
        self.assertEqual(self.wash.state_sequence(), FULL_CYCLE)

    def test_pwm_levels(self):
        self.assertEqual(pwm_during(self.wash, 9), {(0x00, 0xFF)})    # brush 100 %
        self.assertEqual(pwm_during(self.wash, 18), {(0xFF, 0x26)})   # dryer 85 %

    def test_program_led(self):
        self.assertIn('LED_INTENSE', turned_on(self.wash))
        self.assertNotIn('LED_NORMAL', turned_on(self.wash))


class TwoCarsInARow(unittest.TestCase):
    """Regression test for flags that were not reset between cars (fixed in v1.1)."""

    @classmethod
    def setUpClass(cls):
        cls.wash, cls.cpu = simulate([Car(intense=False), Car(intense=True)])

    def test_both_cars_complete_the_cycle(self):
        self.assertEqual(self.wash.cars_done, 2)
        self.assertEqual(self.wash.state_sequence(), FULL_CYCLE + FULL_CYCLE[1:])

    def test_second_car_waits_for_both_distances_in_e8(self):
        first, second = self.wash.time_in_state(8, 0), self.wash.time_in_state(8, 1)
        self.assertAlmostEqual(first, second, delta=0.3)


class StaticChecks(unittest.TestCase):
    """Checks on the source itself, no simulation needed."""

    @classmethod
    def setUpClass(cls):
        from run_sim import FIRMWARE
        from sim8051 import Program, CPU
        from plant import CarWash
        cls.prog = Program(FIRMWARE)
        cls.cpu = CPU(cls.prog, CarWash([]))

    def test_every_bit_symbol_has_its_own_address(self):
        seen = {}
        for name, value in self.prog.equ.items():
            if '.' not in value:
                continue
            addr = self.cpu._bit_addr(name)
            self.assertNotIn(addr, seen, f'{name} and {seen.get(addr)} share bit address {addr:#04x}')
            seen[addr] = name

    def test_state_table_has_21_entries(self):
        start = self.prog.labels['STATE_TABLE']
        targets = [a[0] for op, a, *_ in self.prog.ins[start:start + 21]]
        self.assertEqual(len(targets), 21)
        for n, label in enumerate(targets):
            self.assertTrue(label.startswith(f'E{n}_'), label)


class Timeouts(unittest.TestCase):
    def test_alarm_after_30_s_without_token_and_reset_when_car_leaves(self):
        wash, cpu = simulate([Car(token_after=None, start_after=None, leave_after=40)], max_seconds=60)
        arrival = next(t for t, s in wash.transitions if s == 1)
        alarm = wash.edges('ALARM_START', 1)[-1]
        self.assertAlmostEqual(alarm - arrival, 30.0, delta=0.1)
        self.assertEqual(wash.state_sequence(), [0, 1, 0])
        self.assertEqual(cpu.symbol_bit('ALARM_START'), 0)
        self.assertEqual(cpu.symbol_bit('LIGHT_GREEN'), 1)
        self.assertEqual(cpu.symbol_bit('TR0'), 0)

    def test_alarm_after_30_s_and_auto_start_after_60_s_without_start_button(self):
        wash, cpu = simulate([Car(token_after=2.0, start_after=None)], max_seconds=200)
        token = next(t for t, s in wash.transitions if s == 2)
        alarm = wash.edges('ALARM_START', 1)[-1]
        start = next(t for t, s in wash.transitions if s == 3)
        self.assertAlmostEqual(alarm - token, 30.0, delta=0.1)
        self.assertAlmostEqual(start - token, 60.0, delta=0.1)
        self.assertEqual(wash.state_sequence(), FULL_CYCLE)


if __name__ == '__main__':
    unittest.main(verbosity=2)
