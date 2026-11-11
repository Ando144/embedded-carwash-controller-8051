"""Run the car wash firmware in the simulator and print what happens.

    python tools/sim/run_sim.py                 # one car, normal program
    python tools/sim/run_sim.py --intense       # one car, intense program
    python tools/sim/run_sim.py --cars 2        # two cars in a row
    python tools/sim/run_sim.py --plot out.png  # also draw the wash cycle (needs matplotlib)
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim8051 import CPU, Program          # noqa: E402
from plant import Car, CarWash            # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
FIRMWARE = os.environ.get('CARWASH_FIRMWARE', os.path.join(ROOT, 'src', 'carwash_controller.asm'))

STATE_NAMES = ['Idle', 'Wait token', 'Wait start', 'Approach', 'Soap', 'Position', 'Wash rear',
               'Place rollers', 'Approach rollers', 'Wash', 'Raise H-roller', 'Position',
               'Wash front', 'Place rollers', 'Position for rinse', 'Rinse', 'Position 40 cm',
               'Approach car', 'Dry', 'Return home', 'Blink']


def simulate(cars, firmware=FIRMWARE, max_seconds=300):
    wash = CarWash(cars)
    cpu = CPU(Program(firmware), wash)
    n = len(cars)
    cpu.run(max_seconds, stop=lambda: wash.cars_done >= n and wash.state == 0
            and wash.t > wash.transitions[-1][0] + 0.5)
    return wash, cpu


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--intense', action='store_true', help='intense program (TOKEN_TYPE = 1)')
    ap.add_argument('--cars', type=int, default=1, help='number of cars in a row')
    ap.add_argument('--firmware', default=FIRMWARE, help='path to the .asm file')
    ap.add_argument('--plot', metavar='FILE', help='save a chart of the wash cycle')
    args = ap.parse_args()

    cars = [Car(intense=args.intense if i == 0 else not args.intense) for i in range(args.cars)]
    t0 = time.time()
    wash, cpu = simulate(cars, args.firmware)
    print(f'{"time":>8}  state')
    for t, s in wash.transitions:
        print(f'{t:7.2f}s  E{s:<2} {STATE_NAMES[s]}')
    pwm = sorted({(s.pwm0, s.pwm1) for s in wash.samples if s.t > 0})
    e9 = [s for s in wash.samples if s.state == 9 and 0 <= s.x <= 450]
    print(f'\ncars washed: {wash.cars_done}   simulated: {wash.t:.1f} s   '
          f'wall time: {time.time() - t0:.1f} s')
    print('PWM0 / PWM1 values used:', ', '.join(f'0x{a:02X}/0x{b:02X}' for a, b in pwm))
    if e9:
        print(f'E9 brushing: side distance {min(s.h_dist for s in e9):.1f}-'
              f'{max(s.h_dist for s in e9):.1f} cm, roof distance '
              f'{min(s.v_dist for s in e9):.1f}-{max(s.v_dist for s in e9):.1f} cm')
    if args.plot:
        from plot_cycle import plot
        plot(wash, args.plot)
        print('chart saved to', args.plot)


if __name__ == '__main__':
    main()
