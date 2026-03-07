# Final Code Review Report: mazerunner-core

**Project:** mazerunner-core-ukmars
**Platform:** ATmega328P / Arduino Nano, 16 MHz, 2 KB SRAM, 32 KB Flash
**Review date:** 2026-03-07
**Source files reviewed:** All files under `mazerunner-core/`
**Prior documents:** `review/01-structural-survey.md`, `review/02b-hardware-abstraction.md`,
`review/02d-state-machines.md`, `review/03a-memory-safety.md`, `review/03b-error-handling.md`,
`review/03c-timing.md`

---

## 1. Executive Summary

mazerunner-core is a complete, competition-ready embedded software stack for a half-size
micromouse robot, implementing trapezoidal motion profiling, PD position control with three-term
feedforward, interrupt-driven double-sampling ADC sensor sequencing, BFS flood-fill maze solving,
and a serial CLI — all on an ATmega328P with 2 KB of SRAM and no operating system.
The codebase reflects extensive domain experience: the ISR architecture, motor mathematics, and
maze algorithms are sophisticated and clearly the product of iteration over real hardware.
Code quality is consistently above the Arduino hobbyist baseline, with careful use of ATOMIC
guards, direct register manipulation where needed, and meaningful inline documentation.
The most important finding is a silent navigation failure in `mouse.h:426-427`: when
`maze.heading_to_smallest()` returns `BLOCKED` (value 99), the caller converts this to a
turn direction via `(99 - m_heading) & 0x3`, which maps to LEFT for any heading — causing
a walled-in or confused robot to spin indefinitely in a corner with no diagnostic output and
no way to abort from the main loop.

---

## 2. Architecture Assessment

### Fitness for purpose

The architecture is appropriate for its constraints. The two-tier model — a 500 Hz ISR tier
responsible for all time-critical control (encoders, profiling, PD motor control, sensors) and
a main-loop tier responsible for navigation logic — is the correct approach for a bare-metal
embedded system where motor safety must not be at the mercy of application latency. The use of
`ISR_NOBLOCK` on the systick ISR to allow encoder preemption is a subtle but correct choice
that prevents count loss without requiring a separate high-priority encoder ISR.

### Separation of concerns

Subsystem boundaries are well-drawn. Each driver (`Encoders`, `AnalogueConverter`, `Sensors`,
`Motors`) owns its hardware exclusively; the systick orchestrator (`Systick::update()`) calls
them in dependency order. Navigation logic (`Mouse`) sits entirely above the control layer and
knows nothing about hardware registers.

The main violation of separation is the CLI (`cli.h`): it directly references `battery`,
`maze`, `mouse`, `encoders`, `sensors`, `motion`, and `reporter` rather than operating through
a defined interface. For a project of this size this is entirely workable, but it means the
CLI is tightly coupled to every subsystem.

The `Mouse` class is the largest single unit of complexity: it contains navigation algorithms,
sensor-based UI (hand-start detection), calibration tools, and test routines. These are
functionally distinct concerns occupying the same class. The commented `TODO` at `mouse.h:34`
("this code combines Robot with Mouse") acknowledges this.

### Header-only, single translation unit

All class definitions, implementations, and global data declarations reside in `.h` files
included from a single `.ino` translation unit. This is idiomatic for Arduino projects of this
scale and imposes no practical disadvantage here. It means the compiler sees the entire program
at once, enabling aggressive inlining and constant-folding — beneficial on a constrained target.
The trade-off is that any change to any header triggers a full recompile.

### What would be hardest to change

1. **The ADC channel mapping and sensor board layout.** The ADC sequencer always converts all
   8 channels in a fixed order (`adc.h:188-229`), tightly coupled to the sensor board pinout.
   Adding a 9th channel (e.g., IMU via SPI interrupt) would require restructuring the ISR state
   machine.

2. **The single float-based motion profiler.** `Profile` is unit-agnostic but not composable:
   there are exactly two instances (`forward`, `rotation`), declared as global externs. Adding
   a third axis (e.g., for a pan sensor) or parameterising the profiler count at compile time
   would require architectural changes.

3. **The CLI command set.** The `commands[]` PROGMEM table (`cli.h:30`) and the hard-coded
   switch statement in `run_long_cmd()` (`cli.h:329-344`) must be modified in parallel. There
   is no registration mechanism.

---

## 3. Key Design Decisions

| Decision | Rationale (inferred) | Assessment |
|---|---|---|
| `ISR_NOBLOCK` on systick (`mazerunner-core.ino:57`) | Encoder ISRs must not be blocked during the 700–800 μs systick ISR or counts are lost at high speed | **Appropriate** — correct solution to a real problem; clearly understood by the author |
| All state in globals, no heap (`mazerunner-core.ino:31-44`) | 2 KB SRAM makes `malloc` impractical; fragmentation on a long-running system would be fatal | **Appropriate** — correct choice for this platform; SRAM budget is tight but managed |
| `PERSISTENT` / `.noinit` for Maze (`config.h:142`) | Preserve maze map across warm resets so a partial search can be resumed | **Appropriate** — clever use of linker section; requires cold-start discipline (button-hold to clear) |
| Header-only, single `.ino` translation unit | Arduino build model simplicity; no separate `.cpp` files to manage | **Appropriate** at this scale — enables LTO-equivalent inlining; would not scale beyond ~3,000 lines |
| Software float for all control arithmetic | ATmega328P has no FPU; only option for proportional/feedforward math | **Questionable** — many controller terms could be integer or fixed-point, reducing systick load from ~40% to ~20% |
| ADC double-sampling (dark + lit, `adc.h:83-229`) | Subtract ambient light contribution; single emitter pulse per sample | **Appropriate** — standard technique; the 5-phase interrupt-driven sequencer fits in ~620 μs with only 100 μs CPU time |
| BFS flood with stack-allocated `Queue<Location,64>` (`maze.h:430`) | No heap; bounded queue avoids dynamic allocation | **Questionable** — queue bound of 64 is the author's own estimate, explicitly described as "unproven"; silent overflow produces corrupt cost maps |
| Three-level compile-time config hierarchy: hardware → robot → event (`config.h:63-128`) | Multiple robots and venues; no runtime cost for unused config | **Appropriate** — clean and zero-overhead; event-specific calibration at `config-robot-orion.h:56-148` demonstrates practical utility |

---

## 4. Notable Features

**Encoder ISR design with `ISR_NOBLOCK`.** Making systick re-entrant so that `INT0`/`INT1`
encoder ISRs can fire mid-systick is the right solution to encoder count loss at high motor
speeds. The ~3 μs encoder ISR (per `encoders.h:81` comment) is fast enough to not meaningfully
perturb systick completion time.

**Double-sampling ADC with ambient rejection.** The 5-phase sequencer (DARK_READ → EMITTER_ON
→ SETTLE → LIT_READ → COMPLETE) in `adc.h:184-230` correctly subtracts ambient illumination
from wall sensor readings without requiring any additional optical filtering. The SETTLE phase
(one conversion cycle to let IR emitters stabilise before sampling) shows careful attention to
hardware behaviour.

**Unit-agnostic `Profile` class.** The same trapezoidal profiler (`profile.h`) drives both
forward motion (mm units) and rotation (degree units) with no code duplication. The class
correctly handles reverse motion, zero-distance edge cases (`m_state = PS_FINISHED` for
`distance < 1.0`), and floating-point rounding at the finish line.

**Three-term motor feedforward.** `Motors::leftFeedForward()` (`motors.h:146-159`) combines
speed feedforward, acceleration feedforward, and a static bias term to overcome stiction. This
is more principled than the typical "just add I-term" approach and the derivation methodology
is documented both in comments and via links to external videos.

**Maze stored twice with symmetric update.** `Maze::set_wall_state()` (`maze.h:500-522`) always
writes both the cell's own wall record and its neighbour's mirror wall in a single call. This
eliminates an entire class of inconsistency bugs that arise when wall state is stored once and
mirrored only at read time.

**`ATOMIC` macro as AVR abstraction.** `config-ukmarsbot.h:119` wraps `ATOMIC_BLOCK
(ATOMIC_RESTORESTATE)` behind `ATOMIC`, and defines it as empty on non-AVR targets. This makes
the interrupt-guard intent visible in the code without platform-specific noise.

**Compile-time sensor calibration.** Scale factors (`FRONT_LEFT_SCALE`, `LEFT_SCALE`, etc.)
are computed at compile time from raw calibration constants (`config-robot-orion.h:344-347`).
The runtime sensor update (`sensors.h:218-221`) does only a single multiply per channel, with
no division.

**Absolute scheduling in `Reporter`.** The `s_report_time += s_report_interval` pattern
(`reporting.h:165`) is the correct absolute-tick approach to periodic output. Drift-free
scheduling in a simple polling loop is easy to get wrong; this implementation gets it right.

---

## 5. Risk Register

All issues from all prior review phases, deduplicated and sorted by severity.

| ID | Area | Issue | Severity | File / Location | Recommendation |
|---|---|---|---|---|---|
| FR-01 | Navigation | `BLOCKED` heading (value 99) aliased to `LEFT` by `(99 - m_heading) & 0x3`; walled-in robot spins indefinitely, no diagnostic | ~~**HIGH**~~ ✓ | `mouse.h:426-427` | Fixed — `panic()` updated to halt and disable drive; BLOCKED guard added in `search_to()` and `search_maze()`; closes #16 |
| FR-02 | Safety | No watchdog timer; infinite blocking loops (profiler wait, front-wall creep, spin\_turn) leave motors powered with no automatic recovery | **HIGH** | `profile.h:131`, `mouse.h:96-109`, `motion.h:152` | Enable AVR watchdog (`wdt_enable(WDTO_500MS)`); feed it only from systick |
| FR-03 | Memory | `buf[20]` in `handle_search_command()` is too small for `sprintf_P("Search to %d,%d\n", x, y)` — max output is 23 bytes; stack corruption on two-digit coordinates | **HIGH** | `cli.h:356-357` | Increase to `buf[24]` or use `snprintf_P` with size |
| FR-04 | Memory | `args.argv[1]` and `args.argv[2]` accessed unconditionally without checking `args.argc`; UB when `SEARCH` is invoked with missing arguments | **HIGH** | `cli.h:349, 353, 403` | Guard with `if (args.argc < 2)` before each argv access |
| FR-05 | Memory | `adc.get_dark()` / `get_lit()` return `int` (16-bit on AVR) from ISR-shared arrays without ATOMIC guard; torn reads possible | **HIGH** | `adc.h:158-164`; callers `battery.h:39`, `switches.h:52` | Wrap in `ATOMIC` or return the result of a local copy taken under interrupt disable |
| FR-06 | Config | `DEGREES_PER_RADIAN = 360.0 / 2 * PI` evaluates to ~565.5 due to left-to-right evaluation; correct value is ~57.3 | **HIGH** | `config.h:36` | Fix to `360.0 / (2 * PI)`; or remove — the constant is unused |
| FR-07 | Timing | Systick ISR at 35–40% CPU (700–800 μs / 2000 μs) with two active profiles; headroom is thin and not instrumented | **HIGH** | `systick.h:52-62` | Restore GPIO instrumentation pin; convert inner-loop integer arithmetic from float where possible |
| FR-08 | Control | `Profile::update()` continues executing in `PS_FINISHED` state (speed tracking towards target still runs); state name implies no further work | **MEDIUM** | `profile.h:201-243` | Add `if (m_state == PS_FINISHED) return;` or guard the speed-tracking block |
| FR-09 | Control | `g_steering_mode` initialises to `STEER_NORMAL` in the class declaration, not `STEERING_OFF`; steering runs with uninitialised sensor data before `Mouse::init()` is called | **MEDIUM** | `sensors.h:106` | Change default to `STEERING_OFF`; steering must be explicitly enabled |
| FR-10 | Safety | Button abort checked only at top of `while (m_location != target)` navigation loop; no abort inside `wait_until_finished()`, `wait_until_position()`, or front-wall approach loops | **MEDIUM** | `mouse.h:417, 279, 519`; `profile.h:131`; `motion.h:204` | Pass a button-check predicate into blocking waits, or add a `volatile bool g_abort` checked inside inner loops |
| FR-11 | Logging | `Reporter::set_printer()` is a broken no-op; `static Stream& printer = Serial` cannot be rebound by reference assignment; all output always goes to `Serial` | **MEDIUM** | `reporting.h:98, 108-110` | Change `printer` to a `Stream*` pointer; `set_printer(Stream& s)` stores `printer = &s` |
| FR-12 | Safety | Battery voltage reads as 0.0 on first systick tick (ADC not yet sampled); if `enable_controllers()` is called before ADC has run, `pwm_compensated()` divides by zero | **MEDIUM** | `motors.h:222`; `battery.h:38-41` | Clamp denominator: `max(1.0f, battery_voltage)` inside `pwm_compensated()` |
| FR-13 | Memory | `volatile SensorChannel lfs/rss/lss/rfs` struct fields read field-by-field from main context without ATOMIC; 16-bit `raw` and `value` may tear | **MEDIUM** | `sensors.h:95-98`; callers in `reporting.h`, `mouse.h:885-891` | Take a local snapshot under `ATOMIC` before using both fields of the same channel |
| FR-14 | Safety | `show_adc()` contains `while(true)` with no exit path; `sensors.disable()` after the loop is unreachable; emitters stay on until power cycle | **MEDIUM** | `reporting.h:399-413` | Add button-press escape: `while (!switches.button_pressed())`; move `sensors.disable()` inside loop exit |
| FR-15 | Safety | Front-wall creep loops in `stopAndAdjust()` and `stop_at_center()` have no timeout and no button escape; robot pushes against wall under power indefinitely if sensor threshold not reached | **MEDIUM** | `mouse.h:105-109`, `200-207` | Add a countdown limit (e.g., 200 × 2 ms = 400 ms) and break on timeout |
| FR-16 | Maze | `Queue::add()` silently drops items on overflow; BFS flood produces incorrect cost values if frontier exceeds 64 cells; queue bound is "unproven" by own comment | **MEDIUM** | `maze.h:427-432`; `queue.h` | Prove the bound or increase queue to `MAZE_CELL_COUNT`; add an overflow flag and propagate it to the caller |
| FR-17 | Maze | Cold power-on leaves `Maze::m_mask` and `m_walls` with random `.noinit` data; only the button-hold-at-reset mechanism prevents use of corrupt map | **MEDIUM** | `config.h:142`; `maze.h:247-252` | Store a magic number in a separate `.noinit` variable; check it on `setup()` and force `maze.initialise()` if absent |
| FR-18 | Timing | `maze.flood()` (~7 ms with ISR preemption) is called once per navigation cell in `search_to()` during live motion; delays button-abort processing by ~7 ms per cell | **MEDIUM** | `mouse.h:425`; `maze.h:417` | Acceptable as-is for typical mazes; document the worst-case delay explicitly |
| FR-19 | Timing | ADC ISR uses `digitalWrite()` (~5 μs) for emitter control; `encoders` ISR uses `fast_write_pin()` (~60 ns); inconsistent and adds ~20 μs of unnecessary latency per cycle | **MEDIUM** | `adc.h:198-200, 225-226` | Replace `digitalWrite(emitter_*(), v)` with `fast_write_pin(m_emitter_*_pin, v)` |
| FR-20 | Timing | `Serial.print()` calls in navigation loop block main loop for 1–5 ms when TX buffer fills; delays button abort during verbose search | **MEDIUM** | `mouse.h:406-412, 529, 616` | Reduce logging to one line per cell, or add a `DEBUG_LOGGING` compile-time guard as the commented `#define` suggests (`config-robot-orion.h:196`) |
| FR-21 | Control | `Mouse::panic()` is defined but never called; no code path reaches it; intended as a safe-state handler | ~~**MEDIUM**~~ ✓ | `mouse.h:708-714` | Fixed — `panic()` wired to BLOCKED error paths in FR-01 fix; panic() itself updated to halt and disable drive |
| FR-22 | Memory | `Mouse::State` enum declared but no member variable of that type exists; all values are dead code | **MEDIUM** | `mouse.h:39` | Remove the enum or implement the state machine it was intended to represent |
| FR-23 | Hardware | `fast_write_pin`/`fast_read_pin` silently fall through to `digitalWrite`/`digitalRead` on non-ATmega328 targets; encoder ISR timing degrades to ~30 μs per call | **LOW** | `config-ukmarsbot.h:91-92` | Add `#warning` or `#error` on unsupported targets rather than degrading silently |
| FR-24 | Memory | `encoders.m_total_left` / `m_total_right` are `int` (16-bit); overflow at ~8.5 m continuous travel; documented as test-only but accessed by `cli.h:429` | **LOW** | `encoders.h:238-239` | Change to `int32_t`, or assert/clamp in the CLI formatter |
| FR-25 | Memory | Unguarded reads of `volatile float m_robot_distance`, `m_robot_angle` in some Reporter paths that do not call the accessor methods | **LOW** | `reporting.h:166-168`, `282-283` | Use `encoders.robot_distance()` / `robot_angle()` (which add ATOMIC guards) instead of direct field access |
| FR-26 | Hardware | `m_emitter_front_pin` initialised to `uint8_t(-1)` = 255; `set_front_emitter_pin()` is called from `begin()`, but any call to `emitter_front()` before `begin()` returns 255 | **LOW** | `adc.h:235` | Initialise to a sentinel value that cannot be a valid pin (e.g., `0xFF`) and assert in `emitter_front()` that `begin()` was called |
| FR-27 | Timing | `pwm_compensated()` performs a float divide inside systick ISR; `battery_voltage` changes at <1 Hz — caching `1.0f / battery_voltage` would save ~45 μs/tick | **LOW** | `motors.h:222` | Cache reciprocal in `battery.update()` and expose as `battery.inv_voltage()` |
| FR-28 | Timing | ISR jitter due to `ISR_NOBLOCK` encoder preemption (up to ~120 μs at max speed) is not accounted for by the fixed `LOOP_INTERVAL` constant; causes small velocity estimation error | **LOW** | `config-robot-orion.h:242` | Document the error bound; acceptable for this application class |
| FR-29 | Config | No `static_assert` validates configuration constants; e.g., nothing verifies `MOTOR_MAX_PWM <= 255`, `LOOP_FREQUENCY == 500`, or that queue size divides `MAZE_CELL_COUNT` | **LOW** | `config-robot-orion.h` | Add `static_assert` guards for the most critical invariants |
| FR-30 | Hardware | ADC prescaler set to 32 (→ 500 kHz clock, `adc.h:127-130`); ATmega328P datasheet specifies ≤200 kHz for full 10-bit accuracy; effective resolution is ~8 bits at 500 kHz | **LOW** | `adc.h:127-130` | Document intentional reduced resolution; verify sensor thresholds are calibrated with this ADC speed |
| FR-31 | Error | `Switches::read()` returns -1 on out-of-range ADC; no call site checks for this; `button_pressed()` treats it as "not pressed" which is safe but masks hardware faults | **LOW** | `switches.h:73`; all callers | Add a `is_valid()` check or log when `read()` returns -1 |
| FR-32 | Error | `attachInterrupt()` silent failure if encoder pins (D2, D3) are not valid interrupt pins on target board; encoders would never update | **LOW** | `encoders.h:56-57` | `static_assert(digitalPinToInterrupt(ENCODER_LEFT_CLK) != NOT_AN_INTERRUPT, ...)` |

---

## 6. Questions for the Author

1. **Queue bound.** `maze.h:427` says the maximum BFS frontier for a 16×16 maze "is believed
   to be 64 — HOWEVER, this is unproven." Has this been tested against the full set of legal
   UKMARS maze configurations, or only empirically against a few real mazes? A single adversarial
   maze design (e.g., a checkerboard) can create very wide frontiers.

2. **BLOCKED heading.** The `BLOCKED` sentinel (value 99) from `heading_to_smallest()` is
   never handled by any of its three callers (`search_to()`, `print_maze()`, `search_maze()`).
   Was there an intention to add "walled in" detection that was left incomplete? What should the
   robot actually do in that situation?

3. **`Mouse::panic()`.** The function exists, flashes LEDs, and waits for a button press — a
   reasonable safe-state response. It was removed from all call sites at some point. Was its
   removal intentional (e.g., the LED behaviour was confusing during contests), or is it meant
   to be wired back into the error paths?

4. **`Mouse::State` enum.** `FRESH_START`, `SEARCHING`, `INPLACE_RUN`, `SMOOTH_RUN`,
   `FINISHED` are declared but the state machine they imply is not implemented. Is this
   planned, partially implemented elsewhere, or abandoned?

5. **`set_printer()`.** The class comment in `reporting.h:36-58` describes redirecting reporter
   output to a Bluetooth module or logger as a practical workflow. Given that this has never
   worked (reference assignment is a no-op), has the BT output feature actually been used?
   If so, how?

6. **ADC clock speed.** The prescaler change (128 → 32) brings the ADC to 500 kHz. The
   ATmega328P datasheet specifies ≤200 kHz for guaranteed 10-bit accuracy. At 500 kHz the
   effective resolution is approximately 8 bits. Were the sensor calibration thresholds in
   `config-robot-orion.h` measured with the ADC running at 500 kHz? If the ADC were slowed to
   200 kHz, would the 620 μs sequence still fit within the 2 ms tick?

7. **Steering on startup.** `Sensors::g_steering_mode` defaults to `STEER_NORMAL` in the class
   declaration (`sensors.h:106`). Between construction of the `Sensors` global and the first
   call to `Mouse::init()` (which sets it to `STEERING_OFF`), the steering controller runs on
   uninitialised sensor data. Has this caused any observable startup transient? Is the intent
   for the default to be `STEERING_OFF`?

8. **`search_maze()` return value.** The function signature promises `int` but always returns
   0 (`mouse.h:682`). Was there a plan for this to return a meaningful completion code — e.g.,
   whether the maze is fully solved and the speed-run route avoids unvisited cells?

9. **Cold power-on maze integrity.** The only mechanism to detect that a `.noinit` maze
   contains garbage (vs. a valid prior search) is the button-hold-at-reset gesture. During
   a competition, is this sufficiently reliable? Has the robot ever started a run on a corrupt
   maze after an accidental power cycle?

10. **Systick float budget.** The comment in `systick.h:52-62` reports 35–40% CPU load with
    two active profiles. This was presumably measured on real hardware. Was it measured with
    the profiler running straight (no turns), or during combined forward + rotation motion?
    Combined motion stresses the feedforward terms most heavily.

11. **`do_conversion()`.** `adc.h:176-182` implements a blocking single-channel ADC conversion
    that is unsafe if called while the interrupt-driven sequencer is active. It is currently
    unreachable (no call site exists). Is it retained as a debugging utility? If so, should it
    disable the ADC interrupt before use and restore it after?

12. **Turn parameters.** `config-robot-orion.h:361-367` defines four turn types, but `SS90L`
    and `SS90R` (indices 2 and 3) have identical parameters to `SS90EL` and `SS90EL`
    respectively. Are SS90L/SS90R intended to be distinct turns (e.g., for a different cell
    geometry) that haven't been characterised yet, or are they always identical to the SS90E
    variants by design?
