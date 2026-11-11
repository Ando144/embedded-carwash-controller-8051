# Changelog

## [1.1.0] — 2026-11-11 — Bug-fix release

Found during the code review that followed the English translation. Each fix was checked by
running the firmware, with and without the fix, in the simulator now included in
[`tools/sim`](tools/sim) (an instruction-level 80C552 model coupled to a model of the car wash).
The fixed version completes two full wash cycles in a row (normal and intense program).

### Fixed

| # | Problem in v1.0 | Effect | Fix |
|---|-----------------|--------|-----|
| 1 | `MOV A,0x14` in `Unit_Conversion` compared `cnt50ms` with the **content of RAM address 0x14**, not with the number 20 | The seconds counter advanced only every 256 × 50 ms = 12.8 s: a "4 s" delay lasted ≈ 51 s, and the 30 s / 60 s timeouts took minutes | Tick generation rewritten (`Unit_Conversion`, `Flag_Compare`): 1 s = 20 × 50 ms, `tick100ms` every 2 interrupts, 4 / 30 / 60 s ticks set once |
| 2 | In E9, ADC1 (height) was only requested when the side distance (ADC0) needed a correction | The horizontal roller was not controlled while the sides were in range: in simulation it came within 4 cm of the roof, then rose to the top and stayed there | New event 11: ADC1 is read after every ADC0 reading |
| 3 | `FLAG0` and `IN_PIZTU` were both defined as bit `0x21.2` | Setting one silently changed the other (E7 ↔ E9) | Own bit `0x21.6`, renamed `bridge_reversed` |
| 4 | `bridge_reversed`, `dist_ok` and `height_ok` were never cleared after a car | For the next car, E8 skipped the side-distance check (`dist_ok` still 1), and E7 could fail to reverse the bridge | Cleared when entering E8 (and in `Init`) |
| 5 | Initial state was `0x07` (E7), a leftover from testing | After reset the machine started in the middle of the cycle | Initial state `0x00` (E0 Idle) |
| 6 | `E20_ACT2` had no `RET` and fell through into `Timer_Start` | Timer 0 was restarted right after being stopped when returning to E0 | `RET` added |
| 7 | `CLR cnt50ms` / `CLR cnt1s` in E1 (bit instruction on a byte variable) | Cleared bits 0x22 / 0x23 (RAM byte 0x24) instead of the counters, so the 30 s / 60 s of E2 were not counted from the token | `MOV cnt50ms,#0x00` / `MOV cnt1s,#0x00` |
| 8 | PWM duty values did not match the specification in the report: `PWM0_On` loaded 0x80 (50 %) for both programs, and `PWM1_On` used 85 % / 100 % instead of 50 % / 85 % | The intense program brushed at the same power as the normal one; the dryer ran stronger than specified | Report values: brush 50 % / 100 % (0x80 / 0x00), dryer 50 % / 85 % (0x80 / 0x26) |
| 9 | E13 did not stop `VR_OUT` at the edge limit switch | Roller actuator left powered until the next car | `CLR VR_OUT` in `E13_ACT1` |
| 10 | `PWMP = 0x18` (24): the report rounded 23.53 before subtracting 1 | PWM ran at ≈ 941 Hz instead of 1 kHz | `PWMP = 0x17` → ≈ 980 Hz |
| 11 | `*_ON` correction flags of E9 not cleared when leaving E9 | A stale flag could skip the first correction on the next car | Cleared in `E9_ACT9` |

### Added

- `tools/sim/`: instruction-level 80C552 simulator that runs the A51 source against a model of
  the car wash, plus an automated test suite (18 tests) run by GitHub Actions on every push.
  Each of the 11 bugs above, when put back into the code, makes at least one test fail.
- `docs/img/wash_cycle.png`: chart of a simulated wash cycle (bridge movement and closed-loop
  roller distances).
- MIT license.

### Changed

- `Timer_Start` now always clears the counters and tick flags before starting, so every delay is
  measured from the moment the timer is started.
- E8 / E9 no longer reset `cnt50ms` on each 100 ms tick (no longer needed).
- Removed a duplicated `CLR ALARM_START` in `Init`.

## [1.0.1] — 2026-11 — English translation

- Source code, comments and documentation translated from Basque to English. Identifiers were
  renamed one-to-one. A token-by-token comparison confirmed the translated program is identical
  to the original.
- New README and English design document (`docs/DESIGN.md`); original Basque report kept in
  `docs/original-basque/`.

## [1.0.0] — 2025-01 — Original lab delivery

- Final version of the Computer Architecture lab project (UPV/EHU), written in Basque.
