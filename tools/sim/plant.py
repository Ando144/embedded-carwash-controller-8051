"""Physical model of the car wash tunnel, connected to the simulated 80C552 pins.

Geometry (all distances in cm, speeds in cm/s):

    x     bridge position along the tunnel. Home sensor at x <= -100.
          The car occupies 0 <= x <= 450, so S_CAR = 1 while the bridge is over it.
    g     vertical-roller opening, 0 (closed, centre switch) .. 100 (open, edge switch)
    h     horizontal-roller height, 0 .. 200 (top limit switch at 200)

The car is not a box: its side bulges by 10 cm in the middle and its roof rises by
40 cm, so the rollers have to follow a real profile during the brushing phase (E9).

Distance sensors (50 mV/cm, ADC 0-5 V, 8 bit):
    AIN0  "H" = side distance of the vertical rollers   = g - side(x) + 30
    AIN1  "V" = height of the horizontal roller over roof = h - roof(x)
When the bridge is not over the car both sensors read 100 cm (out of range).
"""
import math
from dataclasses import dataclass, field
from typing import Optional


def pin(port, bit):
    return {'P0': 0x80, 'P1': 0x90, 'P2': 0xA0}[port] + bit


# input pins (same assignment as the EQU block of the firmware)
S_PLATFORM, TOKEN_TYPE, S_TOKEN, BTN_START, S_BRIDGE_HOME = (pin('P0', b) for b in (2, 3, 4, 5, 7))
LS_HR_TOP = pin('P1', 2)
S_CAR, LS_VR_CENTER, LS_VR_EDGE = pin('P2', 3), pin('P2', 4), pin('P2', 5)

OUTPUT_PINS = {
    'LIGHT_GREEN': pin('P0', 0), 'LIGHT_RED': pin('P0', 1), 'ALARM_START': pin('P0', 6),
    'BRIDGE_FWD': pin('P1', 0), 'BRIDGE_BACK': pin('P1', 1), 'HR_UP': pin('P1', 3),
    'HR_DOWN': pin('P1', 4), 'VR_OUT': pin('P1', 6), 'VR_IN': pin('P1', 7),
    'VALVE_WATER': pin('P2', 0), 'VALVE_SOAP': pin('P2', 1),
    'LED_NORMAL': pin('P2', 6), 'LED_INTENSE': pin('P2', 7),
}

CAR_LENGTH = 450.0
HOME = -100.0
BRIDGE_SPEED = 50.0
ROLLER_SPEED = 30.0


@dataclass
class Car:
    """One customer. Times are relative to the moment the car drives onto the platform."""
    intense: bool = False
    token_after: Optional[float] = 2.0      # None = never inserts a token
    start_after: Optional[float] = 4.0      # None = never presses START
    leave_after_done: float = 2.5           # leaves the platform this long after E20 starts
    leave_after: Optional[float] = None     # leave at a fixed time instead (e.g. gives up)


@dataclass
class Sample:
    t: float
    state: int
    x: float
    h_dist: float
    v_dist: float
    pwm0: int
    pwm1: int
    outputs: dict = field(default_factory=dict)


class CarWash:
    def __init__(self, cars, arrive_delay=1.0, sample_period=0.05):
        self.queue = list(cars)
        self.arrive_delay = arrive_delay
        self.sample_period = sample_period
        self.input_pins = {S_PLATFORM, TOKEN_TYPE, S_TOKEN, BTN_START, S_BRIDGE_HOME,
                           LS_HR_TOP, S_CAR, LS_VR_CENTER, LS_VR_EDGE}
        self.x, self.g, self.h = HOME, 100.0, 200.0
        self.t = 0.0
        self._t_last = 0.0
        self._next_sample = sample_period
        self.car = None
        self.car_t0 = None
        self._next_arrival = arrive_delay
        self._leave_at = None
        self.cars_done = 0
        self.state = None
        self.transitions = []      # (t, state)
        self.events = []           # (t, output name, value) on every output change
        self.samples = []          # Sample every sample_period
        self._outputs = {}

    # ---------------------------------------------------------------- geometry
    def over_car(self):
        return self.car is not None and 0.0 <= self.x <= CAR_LENGTH

    def side(self):
        return 10 + 10 * math.sin(math.pi * self.x / CAR_LENGTH)

    def roof(self):
        return 100 + 40 * math.sin(math.pi * self.x / CAR_LENGTH)

    def h_distance(self):
        return self.g - self.side() + 30 if self.over_car() else 100.0

    def v_distance(self):
        return self.h - self.roof() if self.over_car() else 100.0

    # ---------------------------------------------------------------- pins
    def on_platform(self):
        return self.car is not None and (self._leave_at is None or self.t < self._leave_at)

    def read_pin(self, b):
        c, rel = self.car, (self.t - self.car_t0 if self.car is not None else None)
        if b == S_PLATFORM:
            return int(self.on_platform())
        if b == TOKEN_TYPE:
            return int(bool(c and c.intense))
        if b == S_TOKEN:
            return int(bool(c and c.token_after is not None and c.token_after <= rel < c.token_after + 0.5))
        if b == BTN_START:
            return int(bool(c and c.start_after is not None and c.start_after <= rel < c.start_after + 0.3))
        if b == S_BRIDGE_HOME:
            return int(self.x <= HOME)
        if b == LS_HR_TOP:
            return int(self.h >= 200.0)
        if b == S_CAR:
            return int(self.over_car())
        if b == LS_VR_CENTER:
            return int(self.g <= 0.0)
        if b == LS_VR_EDGE:
            return int(self.g >= 100.0)
        raise KeyError(hex(b))

    def adc_read(self, channel):
        d = self.h_distance() if channel == 0 else self.v_distance()
        volts = max(0.0, min(5.0, 0.05 * d))
        return min(255, int(round(256 * volts / 5.0)))

    # ---------------------------------------------------------------- dynamics
    def update(self, cpu):
        self.t = cpu.cycles / 1e6
        dt, self._t_last = self.t - self._t_last, self.t
        out = {name: cpu.read_bit(b) for name, b in OUTPUT_PINS.items()}
        for name, v in out.items():
            if self._outputs.get(name) != v:
                self.events.append((self.t, name, v))
                self._outputs[name] = v

        # customers
        if self.car is None and self.queue and self.t >= self._next_arrival:
            self.car, self.car_t0, self._leave_at = self.queue.pop(0), self.t, None
            if self.car.leave_after is not None:
                self._leave_at = self.t + self.car.leave_after
        if self.car is not None and self._leave_at is not None and self.t >= self._leave_at:
            self.car = None
            self.cars_done += 1
            self._next_arrival = self.t + self.arrive_delay

        # actuators
        self.x += BRIDGE_SPEED * dt * (out['BRIDGE_FWD'] - out['BRIDGE_BACK'])
        self.x = max(HOME - 20, min(CAR_LENGTH + 400, self.x))
        self.g = max(0.0, min(100.0, self.g + ROLLER_SPEED * dt * (out['VR_OUT'] - out['VR_IN'])))
        self.h = max(0.0, min(200.0, self.h + ROLLER_SPEED * dt * (out['HR_UP'] - out['HR_DOWN'])))

        # firmware state
        st = cpu.ram[6]                       # STATE lives in R6
        if st != self.state:
            self.transitions.append((self.t, st))
            self.state = st
            if st == 20 and self.car is not None and self.car.leave_after is None:
                self._leave_at = self.t + self.car.leave_after_done

        if self.t >= self._next_sample:
            self._next_sample += self.sample_period
            self.samples.append(Sample(self.t, st, self.x, self.h_distance(), self.v_distance(),
                                       cpu.ram[0xFC], cpu.ram[0xFD], dict(out)))

    # ---------------------------------------------------------------- queries
    def time_in_state(self, state, occurrence=0):
        """Duration of the n-th visit to `state`."""
        visits = [(t, self.transitions[i + 1][0] if i + 1 < len(self.transitions) else self.t)
                  for i, (t, s) in enumerate(self.transitions) if s == state]
        start, end = visits[occurrence]
        return end - start

    def state_sequence(self):
        return [s for _, s in self.transitions]

    def edges(self, name, value=None):
        return [t for t, n, v in self.events if n == name and (value is None or v == value)]
