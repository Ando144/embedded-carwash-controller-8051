"""Generate the state diagram of the firmware (docs/img/state_machine*.svg) with Graphviz.

    python tools/make_state_diagram.py          # needs the `dot` command (graphviz)

Every event of every state is drawn, as in the original hand-drawn diagram:
  * blue arrows  = transitions to another state
  * grey loops   = events handled inside the same state (alarms, LEDs, ADC readings,
                   closed-loop corrections, blinking)
  * dashed arrows = returns to E0 (car leaves without a token / end of the cycle)
"""
import os
import subprocess

STATES = {
    0: 'Idle', 1: 'Wait token', 2: 'Wait start', 3: 'Approach', 4: 'Soap', 5: 'Position',
    6: 'Wash rear', 7: 'Place rollers', 8: 'Approach rollers', 9: 'Wash', 10: 'Raise H-roller',
    11: 'Position', 12: 'Wash front', 13: 'Place rollers', 14: 'Position for rinse', 15: 'Rinse',
    16: 'Position 40 cm', 17: 'Approach car', 18: 'Dry', 19: 'Return home', 20: 'Blink',
}

PHASES = [
    ('customer', [0, 1, 2]),
    ('pre-wash', [3, 4]),
    ('brushing', [5, 6, 7, 8, 9, 10, 11, 12, 13]),
    ('rinse', [14, 15, 16]),
    ('drying', [17, 18]),
    ('finish', [19, 20]),
]

# (from, to, event, action)
TRANSITIONS = [
    (0, 1, 'S_PLATFORM=1', 'red light, timer on'),
    (1, 0, 'S_PLATFORM=0', 'green light, timer off'),
    (1, 2, 'S_TOKEN=1', 'alarm off, restart counters'),
    (2, 3, 'BTN_START || tick60s', 'bridge forward, timer off'),
    (3, 4, 'S_CAR=1', 'water + soap on'),
    (4, 5, 'S_CAR=0', 'water + soap off, timer on'),
    (5, 6, 'tick4s', 'stop bridge, PWM0 on,\\nrollers in, timer off'),
    (6, 7, 'LS_VR_CENTER=1', 'rollers out'),
    (7, 8, 'S_CAR=1', 'rollers in, H-roller down,\\nclear flags, timer on'),
    (8, 9, 'H and V ok', 'stop roller, bridge back'),
    (9, 10, 'S_CAR=0', 'stop all, H-roller up,\\ntimer off'),
    (10, 11, 'LS_HR_TOP=1', 'bridge back, timer on'),
    (11, 12, 'tick4s', 'stop bridge, rollers in,\\ntimer off'),
    (12, 13, 'LS_VR_CENTER=1', 'rollers out'),
    (13, 14, 'LS_VR_EDGE=1', 'stop rollers, bridge\\nforward, PWM0 off'),
    (14, 15, 'S_CAR=1', 'water on'),
    (15, 16, 'S_CAR=0', 'water off, timer on'),
    (16, 17, 'tick4s', 'bridge back, timer off'),
    (17, 18, 'S_CAR=1', 'PWM1 (dryer) on'),
    (18, 19, 'S_CAR=0', 'PWM1 off'),
    (19, 20, 'S_BRIDGE_HOME=1', 'stop bridge, LEDs off,\\ngreen light, timer on'),
    (20, 0, 'S_PLATFORM=0', 'green light, timer off'),
]

# events that keep the machine in the same state
LOOPS = {
    1: ['tick30s / ALARM_START=1'],
    2: ['tick30s / ALARM_START=1', 'TOKEN_TYPE=0 / LED_NORMAL=1', 'TOKEN_TYPE=1 / LED_INTENSE=1'],
    7: ['LS_VR_EDGE=1 && !bridge_reversed /', '  rollers stop, bridge back'],
    8: ['tick100ms / read ADC0 (H) or ADC1 (V)', 'tick_adc0, H <= 41 cm / stop VR_IN',
        'tick_adc1, V <= 41 cm / stop HR_DOWN'],
    9: ['tick100ms / read ADC0, then ADC1', 'H < 30 cm / VR_OUT    H > 50 cm / VR_IN',
        'V < 30 cm / HR_UP     V > 50 cm / HR_DOWN', 'back at 40 cm / stop that correction'],
    20: ['tick1s / toggle green light'],
}

THEMES = {
    'light': dict(bg='#fcfcfb', ink='#0b0b0b', ink2='#52514e', muted='#898781', band='#f0efec',
                  node='#ffffff', border='#c3c2b7', main='#2a78d6', loop='#898781'),
    'dark': dict(bg='#1a1a19', ink='#f0efec', ink2='#c3c2b7', muted='#898781', band='#262624',
                 node='#1f1f1e', border='#383835', main='#3987e5', loop='#898781'),
}


def dot_source(c):
    out = [
        'digraph fsm {',
        f'  graph [rankdir=TB, bgcolor="{c["bg"]}", fontname="Helvetica", fontsize=11, '
        f'nodesep=1.6, ranksep=0.75, newrank=true, pad=0.3, fontcolor="{c["ink2"]}"];',
        f'  node [shape=box, style="rounded,filled", fillcolor="{c["node"]}", color="{c["border"]}", '
        f'fontname="Helvetica", fontsize=11, fontcolor="{c["ink"]}", penwidth=1.2, margin="0.14,0.06"];',
        f'  edge [fontname="Helvetica", fontsize=8.5, fontcolor="{c["ink2"]}", color="{c["main"]}", '
        f'penwidth=1.6, arrowsize=0.7];',
        f'  start [shape=circle, label="", width=0.16, style=filled, fillcolor="{c["ink"]}", color="{c["ink"]}"];',
    ]
    # U-shaped layout: column A goes down (E0-E6), column B goes up (E7-E13),
    # column C goes down (E14-E20) and E20 returns to E0 across the top.
    cols = [list(range(0, 7)), list(range(13, 6, -1)), list(range(14, 21))]
    phase_of = {s: name for name, states in PHASES for s in states}
    for s in range(21):
        out.append(f'  E{s} [label=<<b>E{s}</b>  {STATES[s]}<br/><font point-size="8" '
                   f'color="{c["muted"]}">{phase_of[s]}</font>>];')
    for col in cols:
        out.append('  ' + ' -> '.join(f'E{s}' for s in col) + ' [style=invis, weight=20];')
    for row in range(7):
        out.append('  { rank=same; ' + '; '.join(f'E{col[row]}' for col in cols) + '; }')
    out.append('  start -> E0 [color="' + c['ink'] + '"];')
    for a, b, ev, act in TRANSITIONS:
        attrs = f'label="{ev}\\n/ {act}"'
        if (a, b) in ((1, 0), (20, 0)):
            attrs += ', style=dashed'
        attrs += ', constraint=false'
        out.append(f'  E{a} -> E{b} [{attrs}];')
    for s, lines in LOOPS.items():
        label = '\\l'.join(lines) + '\\l'
        out.append(f'  E{s} -> E{s} [label="{label}", color="{c["loop"]}", fontcolor="{c["muted"]}", '
                   f'penwidth=1.2, dir=forward];')
    out.append('}')
    return '\n'.join(out)


def main():
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    img = os.path.join(root, 'docs', 'img')
    os.makedirs(img, exist_ok=True)
    for theme, suffix in (('light', ''), ('dark', '_dark')):
        src = dot_source(THEMES[theme])
        path = os.path.join(img, f'state_machine{suffix}.svg')
        subprocess.run(['dot', '-Tsvg', '-o', path], input=src.encode(), check=True)
        if theme == 'light':
            open(os.path.join(img, 'state_machine.dot'), 'w').write(src + '\n')
        print('saved', path)


if __name__ == '__main__':
    main()
