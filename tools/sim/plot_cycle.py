"""Draw a simulated wash cycle: bridge movement by phase, and roller distance tracking.

    python tools/sim/plot_cycle.py docs/img/wash_cycle.png [--dark]
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

THEMES = {
    'light': dict(surface='#fcfcfb', ink='#0b0b0b', ink2='#52514e', muted='#898781',
                  grid='#e1e0d9', axis='#c3c2b7', band='#f0efec', s1='#2a78d6', s2='#eb6834'),
    'dark': dict(surface='#1a1a19', ink='#f0efec', ink2='#c3c2b7', muted='#898781',
                 grid='#2c2c2a', axis='#383835', band='#262624', s1='#3987e5', s2='#d95926'),
}
PHASES = [((3, 4), 'Pre-wash'), ((5, 13), 'Brushing'), ((14, 16), 'Rinse'),
          ((17, 18), 'Drying'), ((19, 20), 'Return')]


def _spans(wash, first, last):
    """Time spans during which the FSM is in states first..last."""
    out, start = [], None
    for t, s in wash.transitions + [(wash.t, -1)]:
        inside = first <= s <= last
        if inside and start is None:
            start = t
        elif not inside and start is not None:
            out.append((start, t)); start = None
    return out


def _style(ax, c):
    ax.set_facecolor(c['surface'])
    for side in ('top', 'right', 'left'):
        ax.spines[side].set_visible(False)
    ax.spines['bottom'].set_color(c['axis'])
    ax.tick_params(colors=c['muted'], labelsize=9, length=0, pad=6)
    ax.grid(axis='y', color=c['grid'], linewidth=0.8)
    ax.set_axisbelow(True)


def plot(wash, path, theme='light'):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    c = THEMES[theme]
    plt.rcParams['font.family'] = ['DejaVu Sans']
    fig, (top, bottom) = plt.subplots(2, 1, figsize=(10, 6.4), dpi=150,
                                      gridspec_kw=dict(height_ratios=[1, 1.1], hspace=0.62))
    fig.patch.set_facecolor(c['surface'])

    # --- top: bridge position over the whole cycle, phases as labelled bands
    t = [s.t for s in wash.samples]
    x = [s.x for s in wash.samples]
    ymax = max(x) + 120
    _style(top, c)
    for k, ((a, b), name) in enumerate(PHASES):
        for s0, s1 in _spans(wash, a, b):
            if k % 2 == 0:
                top.axvspan(s0, s1, color=c['band'], zorder=0, linewidth=0)
            top.text((s0 + s1) / 2, ymax - 25, name, ha='center', va='top', fontsize=9, color=c['ink2'])
    top.plot(t, x, color=c['s1'], linewidth=2, solid_capstyle='round', solid_joinstyle='round')
    top.set_xlim(0, wash.t)
    top.set_ylim(-140, ymax)
    top.set_yticks([-100, 0, 450])
    top.set_yticklabels(['home', 'car front', 'car rear'])
    top.set_xlabel('time (s)', color=c['muted'], fontsize=9)
    top.set_title('Bridge position during one wash cycle', loc='left', fontsize=12,
                  color=c['ink'], pad=12, fontweight='bold')

    # --- bottom: closed-loop distance tracking while brushing (E8-E9)
    e = [s for s in wash.samples if s.state in (8, 9) and 0 <= s.x <= 450]
    t2 = [s.t for s in e]
    _style(bottom, c)
    bottom.axhspan(30, 50, color=c['band'], zorder=0, linewidth=0)
    bottom.axhline(40, color=c['axis'], linewidth=1, zorder=1)
    h = bottom.plot(t2, [s.h_dist for s in e], color=c['s1'], linewidth=2, label='Side (vertical rollers, AIN0)')
    v = bottom.plot(t2, [s.v_dist for s in e], color=c['s2'], linewidth=2, label='Roof (horizontal roller, AIN1)')
    bottom.set_xlim(t2[0], t2[-1])
    bottom.set_ylim(20, 120)
    bottom.set_yticks([20, 40, 60, 80, 100, 120])
    bottom.set_ylabel('distance to car (cm)', color=c['muted'], fontsize=9)
    bottom.set_xlabel('time (s)', color=c['muted'], fontsize=9)
    bottom.set_title('Roller distance while brushing (ADC feedback, every 100 ms)', loc='left',
                     fontsize=12, color=c['ink'], pad=26, fontweight='bold')
    bottom.text(0, 1.03, 'Shaded band: allowed window 30–50 cm   ·   line: 40 cm target',
                transform=bottom.transAxes, fontsize=9, color=c['muted'])
    leg = bottom.legend(handles=h + v, loc='upper right', frameon=False, fontsize=9, ncol=1)
    for txt in leg.get_texts():
        txt.set_color(c['ink2'])
    fig.text(0.125, 0.015, 'Simulated with tools/sim (instruction-level 80C552 model + car wash plant). '
             'Normal program, car 450 cm long with a curved side and roof.',
             fontsize=8, color=c['muted'])
    fig.savefig(path, facecolor=c['surface'], bbox_inches='tight', pad_inches=0.25)
    plt.close(fig)


if __name__ == '__main__':
    from plant import Car
    from run_sim import simulate
    out = sys.argv[1] if len(sys.argv) > 1 else 'wash_cycle.png'
    wash, _ = simulate([Car()])
    plot(wash, out, 'dark' if '--dark' in sys.argv else 'light')
    if '--both' in sys.argv:
        root, ext = os.path.splitext(out)
        plot(wash, root + '_dark' + ext, 'dark')
    print('saved', out)
