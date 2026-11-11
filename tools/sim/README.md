# Firmware simulator and tests

An instruction-level simulator that runs the **real firmware source**
([`src/carwash_controller.asm`](../../src/carwash_controller.asm)) against a physical model of
the car wash. No Keil, no hardware and no external Python packages are needed (matplotlib is
only used to draw the chart).

```
tools/sim/
├── sim8051.py        # 80C552 core: parses the A51 source, executes it cycle by cycle
├── plant.py          # Car wash model: bridge, rollers, car profile, sensors, customers
├── run_sim.py        # Command-line runner: prints the state timeline
├── plot_cycle.py     # Draws docs/img/wash_cycle.png
└── test_firmware.py  # Automated tests (run by GitHub Actions on every push)
```

## Usage

From the repository root (Python 3.9 or newer):

```bash
python tools/sim/run_sim.py                  # one car, normal program
python tools/sim/run_sim.py --intense        # one car, intense program
python tools/sim/run_sim.py --cars 2         # two cars in a row
python -m unittest discover -s tools/sim -v  # run the test suite (a few minutes)
python tools/sim/plot_cycle.py docs/img/wash_cycle.png --both   # regenerate the charts
```

Example output (shortened):

```
    time  state
   0.00s  E0  Idle
   1.00s  E1  Wait token
   3.00s  E2  Wait start
   5.00s  E3  Approach
   ...
  33.37s  E9  Wash
   ...
  82.84s  E20 Blink
  85.34s  E0  Idle

cars washed: 1   simulated: 85.8 s   wall time: 18 s
E9 brushing: side distance 29.9-50.2 cm, roof distance 29.2-50.8 cm
```

## What is modelled

| Part | Model |
|------|-------|
| CPU | The subset of the 8051 instruction set used by the firmware, with data-sheet cycle counts (1 cycle = 1 µs at 12 MHz). Register bank 0, stack, bit-addressable RAM, SFRs. |
| Timer 0 | Mode 1 (16-bit), overflow flag `TF0`, interrupt vector `0x0B`. |
| ADC | Start with `ADCON.ADCS`, channel in `ADCON[2:0]`, result in `ADCH` after 50 cycles, `ADCI` + interrupt vector `0x53`. Sensor: 50 mV/cm, 0–5 V, 8 bit. |
| PWM | `PWM0`, `PWM1` and `PWMP` values are recorded (the waveform itself is not generated). |
| Ports | Input pins are read from the plant; output pins come from the port latches. |
| Plant | Bridge moving at 50 cm/s; rollers at 30 cm/s; limit switches; a 450 cm car whose side bulges 10 cm and whose roof rises 40 cm, so the closed-loop control has a real profile to follow; customers who arrive, insert a token, press START and leave. |

The simulator reads the `.asm` file directly instead of executing a compiled binary, so it
checks the *behaviour* of the program, not the A51 encoding. Building with Keil remains the
reference for that.

## What the tests check

- The machine goes through E0 → E20 → E0 in order, for both programs and for two cars in a row
- The 4 s delays (E5, E11, E16) last 4.0 s, the alarm fires after 30 s, auto-start after 60 s
- While brushing, both rollers stay in the 30–50 cm window as the car profile changes
- PWM duty values per program, program LEDs, 1 Hz green blinking at the end
- After each car everything is switched off and Timer 0 is stopped

Every bug listed in the v1.1 section of [`CHANGELOG.md`](../../CHANGELOG.md) makes at least
one of these tests fail when it is put back into the code.
