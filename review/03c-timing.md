# Review: Timing Correctness and Real-Time Characteristics

**Project:** mazerunner-core-ukmars
**Platform:** ATmega328P @ 16 MHz, no FPU, no RTOS
**Date:** 2026-03-07
**Scope:** All source files in `mazerunner-core/`

---

## Background: Clock and Interrupt Architecture

Four hardware timing resources are in use simultaneously:

| Timer | Rate | Use | Source |
|---|---|---|---|
| Timer0 | ~1 kHz (overflow) | Arduino `millis()` / `delay()` | Arduino framework |
| Timer1 | 31.25 kHz (PWM) | Motor PWM (`CS10`, no prescaler) | `motors.set_pwm_frequency()` — `motors.h:288` |
| Timer2 CTC | **500 Hz** | Systick ISR (`OCR2A=249`, prescaler=128) | `systick.h:34` |
| ADC | 500 kHz | Interrupt-driven sensor sequencer (prescaler=32) | `adc.h:127-132` |

In addition, `INT0` (D2) and `INT1` (D3) fire on `CHANGE` for the two encoder channels.

**Systick calculation:** `16,000,000 / 128 / (249+1) = 500.0 Hz` exactly. Period = 2000 μs.

**ADC clock:** Prescaler 32 → 500 kHz. Each conversion takes 13 ADC clock cycles = **26 μs**. The comment in `adc.h:43` states the full 8-channel double-sampling sequence "takes about 620μs to complete but only uses about 100μs of processor time" (19 ISR firings × ~5 μs/ISR).

---

## 1. Time-Critical Path Inventory

### Path 1 — Systick ISR (TIMER2_COMPA_vect, 500 Hz)

**Deadline:** Must complete in 2000 μs. Must not be re-entered.

**Invocation:** `ISR(TIMER2_COMPA_vect, ISR_NOBLOCK)` — `mazerunner-core.ino:57`

`ISR_NOBLOCK` re-enables the global interrupt flag (`sei`) at the very start of the ISR body,
before any application code runs. This intentionally allows encoder ISRs and the ADC ISR to
preempt systick. It prevents encoder count loss at high speed but makes systick execution time
non-deterministic.

**Worst-case execution sequence** (`systick.h:68-75`):

```
encoders.update()           ← ATOMIC snapshot, float multiply ×4
motion.update()             ← forward.update() + rotation.update() (float math)
sensors.update()            ← adc reads, scale multiply ×4, PD controller
battery.update()            ← adc read, multiply
motors.update_controllers() ← PD + feedforward, 2× analogWrite()
adc.start_conversion_cycle()← starts next ADC sequence
```

**Measured load** (from `systick.h:52-62` comments):

| Condition | CPU fraction | Duration |
|---|---|---|
| Robot at rest | ~10% | ~200 μs |
| One active profile | ~30% | ~600 μs |
| Two active profiles | ~35–40% | **700–800 μs** |

**Available budget:** 2000 μs − 800 μs = **1200 μs** headroom at peak.

**Preemption budget consumed by encoders:** At 400 mm/s search speed, each wheel generates approximately
1534 encoder events/second → ~3 per 2 ms tick per wheel → 6 total. Each encoder ISR is ~3 μs (per
`encoders.h:81` comment) → ~18 μs preemption per tick. At maximum speed (2500 mm/s): ~40 events per
tick → ~120 μs preemption. Still within budget at all expected speeds.

**Preemption budget consumed by ADC ISR:** The ADC sequence is started as the last act of systick
(line 74: "NOTE: no code should follow this line"). With ~19 ADC ISRs per cycle and ~5 μs each,
and given that the sequence from the previous tick always completes before the next tick fires
(480 μs sequence from ~800 μs start < 2000 μs period), there is no overlap between cycles.
ADC ISR preemption within a single systick execution is negligible.

### Path 2 — ADC Conversion ISR (ADC_vect, ~19×/cycle)

**Deadline:** Must complete before the next tick's `start_conversion_cycle()` call.
As established above, the 480 μs hardware sequence fits comfortably within the 2000 μs period.

**Worst-case execution:** The `EMITTER_ON` case calls `digitalWrite()` twice (slow path):

```cpp
// adc.h:198-200
if (m_emitters_enabled) {
    digitalWrite(emitter_diagonal(), 1);
    digitalWrite(emitter_front(), 1);
}
```

`digitalWrite()` on AVR performs pin-to-register lookup and bit manipulation: approximately
**~4–5 μs each**, total ~10 μs per ADC cycle in the EMITTER_ON and COMPLETE cases.
The encoder ISR, by contrast, uses `fast_read_pin()` (a single PORT register access, ~125 ns).
This inconsistency is a **MEDIUM** issue (§4).

### Path 3 — Encoder ISRs (INT0 / INT1, demand-driven)

**Deadline:** Must respond before the next encoder edge arrives (depends on speed).
At 2500 mm/s and 18 magnets per revolution with 11:1 gearbox: approximately 4800 counts/s/wheel.
Edge-to-edge period: ~208 μs. ISR execution: ~3 μs. No deadline concern.

**Critical:** Because systick is `ISR_NOBLOCK`, encoder ISRs can preempt systick mid-flight.
This is correct and intentional — the alternative would be losing counts during the longest ISR.

### Path 4 — Serial TX (main loop, logging during search)

**Timing requirement:** Not safety-critical, but blocking Serial delays button abort.

At 115200 baud (10 bits/char), the 64-byte UART TX buffer drains at **11,520 chars/sec**.
Filling it blocks the caller for up to **5.6 ms**. Each cell during `search_to()` emits
approximately 60–80 characters via `log_action_status()`, `update_map()`, and `Serial.println()`.
If these arrive faster than the UART can drain them (they don't for a single burst), the main
loop stalls. In practice, a single burst of 80 characters exceeds the 64-byte buffer by 16 bytes,
adding ~1.4 ms of blocking per cell.

---

## 2. Blocking in Critical Contexts

### ISR context — clean

No `delay()`, `Serial`, dynamic allocation, flash writes, or unbounded operations occur inside any
ISR. The systick ISR, ADC ISR, and encoder ISRs are all well-disciplined in this respect.

The `analogWrite()` calls inside `motors.update_controllers()` (called from systick) deserve a
note: `analogWrite()` on AVR writes directly to `OCR1A`/`OCR1B` timer registers after
some bookkeeping (~10 instructions, ~1 μs). It is deterministic and bounded. Not a concern.

### Main loop — multiple blocking busy-wait loops

`delay(2)` inside wait loops such as `Profile::wait_until_finished()` (`profile.h:131`),
`Motion::wait_until_position()` (`motion.h:204`), and the front-wall approach loops in
`Mouse::stopAndAdjust()` (`mouse.h:96`) and `stop_at_center()` (`mouse.h:204`) are all in main
loop context. This is by design: the `delay(2)` call allows Timer0 and Timer2 ISRs to fire while
the main loop is parked, which is correct.

However, some `delay()` calls are in the middle of sensor feedback loops where the exit condition
depends on a continuously updated variable:

```cpp
// mouse.h:105-108
if (sensors.see_front_wall) {
    while (sensors.get_front_sum() < FRONT_REFERENCE) {
        motion.start_move(10, 50, 0, 1000);
        delay(2);
    }
}
```

The call `motion.start_move(10, 50, 0, 1000)` re-initiates the forward profiler every 2 ms.
This repeatedly restarts a profile that never completes, which accumulates error in `m_fwd_error`
in the forward controller. This is a functional concern (noted in error-handling review) but
also a timing concern: the main loop consumes CPU every 2 ms calling profile setup while
the ISR is also updating that same profile. No race condition (the profile write is not ATOMIC
here), but the interaction pattern is unusual.

---

## 3. Jitter Sources

### Systick period — exact but execution completion is jittered

Timer2 CTC generates a hardware-exact 2 ms tick. The ISR execution start is precise. Completion
time varies because of `ISR_NOBLOCK` preemption by encoder and ADC ISRs. At normal search speed
(400 mm/s), the jitter is ~18 μs. At maximum speed (2500 mm/s), jitter grows to ~120 μs.

This jitter is not directly visible to the control law because the control calculations use
`LOOP_INTERVAL` (a compile-time constant, `config-robot-orion.h:242`) rather than measuring the
actual elapsed time since the last tick. If the systick completes a few microseconds late, the
controllers still compute as if exactly 2 ms has elapsed. At 120 μs jitter, the time error is
6% of the nominal interval — a small but real source of control inaccuracy at high speeds. **LOW**.

### Profile tick interval — absolute (correct)

The `Profile::update()` in `profile.h:205` advances position by `m_speed * LOOP_INTERVAL` where
`LOOP_INTERVAL` is the compile-time constant `1/500 = 0.002 s`. Position is accumulated without
reference to real time. Because the ISR fires at a hardware-exact rate, the position integrator
is accurate. No drift accumulates. ✓

### Reporter scheduling — absolute (correct)

```cpp
// reporting.h:164-165
if (millis() >= s_report_time) {
    s_report_time += s_report_interval;
```

This is absolute tick scheduling: the next report time is computed from the previous report time,
not from the current time. Even if a report fires slightly late, the schedule does not drift.
This is the correct pattern. ✓

### `delay()` in navigation loops — relative but inconsequential

`delay(2)` in busy-wait loops (`profile.h:132`, `motion.h:206`, `mouse.h:100`) are relative
delays. However, the loop exit conditions are ISR-driven (profile state, sensor value) rather
than time-based, so drift in the `delay()` duration does not affect correctness — it only
affects poll frequency. No drift concern. ✓

### `Switches::button_pressed()` — unbounded poll rate in wait loops

```cpp
// switches.h:80-83
void wait_for_button_press() {
    while (not(button_pressed())) {
        delay(10);
    };
}
```

`button_pressed()` calls `read()` which calls `update()` which calls `adc.get_dark(m_channel)`.
At 10 ms poll interval this is fine. However, `get_dark()` returns the unguarded
`m_adc_dark[m_channel]` (an `int` shared with the ADC ISR), a latent race noted in the
memory safety review.

### `report_radial_track()` — angle-change driven output

```cpp
// reporting.h:228-230
static int recorded_angle = INT16_MAX;
int this_angle = (int)encoders.robot_angle();
if (recorded_angle != this_angle) {
```

This is driven by angle-change rather than a fixed time interval. At very high angular velocity,
one degree changes every fraction of a ms; at rest it never changes. Output rate is therefore
not fixed. No timing concern for the robot, but it means the data stream is irregular during
acceleration phases. **LOW** (documentation note only).

### Shared resource contention

There are no RTOS mutexes or priority inversion mechanisms. Shared state between ISR and main
loop is protected by `ATOMIC_BLOCK`. No ISR calls into a function that could be simultaneously
executing in the main loop. No priority inversion risk.

The one subtle contention point: `Profile::update()` is called from systick and
`Profile::wait_until_finished()` polls `m_state` from the main loop. `m_state` is `volatile`
and `uint8_t` (atomic on AVR), so no guard is needed for that specific read. ✓

---

## 4. Jitter Sources — ISR Length Concerns

### Systick ISR total length at peak load (~800 μs)

The systick ISR is the longest interrupt service routine in the system. At 40% CPU load it
occupies 800 μs. Because it uses `ISR_NOBLOCK`, it cannot starve encoder ISRs. However, it
can delay lower-priority software tasks (main loop) by up to 800 μs at a time.

The Timer0 overflow ISR (Arduino millis, ~1 kHz) can preempt systick via `ISR_NOBLOCK`. Since
Timer0 and Timer2 are independent, a Timer0 interrupt arriving during systick adds ~4 μs to
systick completion. This is negligible.

### Floating-point arithmetic — dominant cost

The ATmega328P has no hardware FPU. AVR-GCC performs all floating-point in software:

| Operation | Typical cycles | Duration @ 16 MHz |
|---|---|---|
| `float` add/sub | ~60–70 | ~4 μs |
| `float` multiply | ~110–140 | ~8 μs |
| `float` divide | ~400–500 | ~28 μs |
| `fabsf()` | ~15–20 | ~1 μs |

The systick ISR performs no divisions. The `pwm_compensated()` function in `motors.h:222`:

```cpp
int pwm = MOTOR_MAX_PWM * desired_voltage / battery_voltage;
```

This performs a float multiply and a **float divide** — the most expensive operation in the ISR.
Called twice per tick (left and right motor), it costs approximately **2 × 28 = 56 μs** per tick
in division alone. This could be replaced with a multiply by the pre-computed reciprocal
`1.0f / battery_voltage`, saving ~45 μs per tick. The battery voltage changes slowly (< 1 Hz
meaningful variation), so caching its reciprocal would be valid.
**LOW** (optimisation opportunity; not currently causing overrun).

### `digitalWrite()` for emitter control in ADC ISR

As noted in §1 Path 2, the ADC ISR uses `digitalWrite()` (slow, ~5 μs) for emitter control
while the encoder ISR uses `fast_write_pin()` (1 cycle, ~62 ns). Each ADC cycle calls
`digitalWrite()` in two phases (EMITTER_ON and COMPLETE), adding ~20 μs of unnecessary latency
to the ADC ISR. Replacing with `fast_write_pin()` would be consistent and marginally faster.
**LOW**.

---

## 5. Timing Instrumentation

### GPIO toggle — present but disabled

Two GPIO instrumentation points are present as comments:

```cpp
// systick.h:65:
// digitalWriteFast(LED_BUILTIN, 1);

// sensors.h:197:
// digitalWriteFast(LED_USER, 1);
```

These appear to be oscilloscope measurement points used during development. They are correctly
commented out for production/competition use. Restoring them for a single-build profiling pass
would allow oscilloscope measurement of systick execution time and sensor update time.

### No software timing counters

No DWT cycle counter (not available on AVR), no software cycle counter, and no timing histogram
are present. The only timing information is the qualitative load percentages recorded as comments
in `systick.h:52-62`. These values are author-measured rather than automatically reported.

### Timing budgets — partially documented

The `systick.h:52-62` comment block explicitly documents measured CPU load percentages for
different operational states. This is valuable and unusual in embedded projects.

No budget exists for the ADC ISR, encoder ISR, or per-cell navigation time. The claim in
`maze.h:427` that "5.3ms when there are no interrupts" is a single-point measurement, not a
formal budget.

---

## 6. Clock and Timekeeping

### Wall time

There is no RTC, GPS PPS, NTP, or other absolute time reference. `millis()` (Arduino Timer0,
1 ms resolution) is the only time base available. It rolls over after ~49.7 days; no code
depends on continuity across rollover.

`millis()` is used exclusively for:

1. Report scheduling in `Reporter` (`s_report_time`, `s_start_time`) — relative elapsed time.
2. Nowhere else in the control or navigation code.

The control law itself uses the compile-time constant `LOOP_INTERVAL = 0.002 s` rather than
measuring actual elapsed time. This is correct for a hardware-timed ISR but means that any
long-term frequency error in the 16 MHz crystal directly biases velocity and position estimates.
For UKMARSBOT at typical contest durations (~30 s), crystal accuracy is not practically limiting.

### `millis()` resolution — adequate for logging, insufficient for fine profiling

At 1 ms resolution, `millis()` cannot resolve events within a single 2 ms tick. The reporter
output includes millisecond timestamps, which means two events in the same tick appear
simultaneous. For high-speed profiling (e.g., measuring sensor stabilisation time within the
ADC settle phase), a cycle counter or sub-ms timer would be required.

### `delay()` accuracy

Arduino's `delay(n)` waits until `millis()` has advanced by `n`. With 1 ms resolution,
`delay(2)` waits for 2–3 ms depending on when within a millisecond it is called. The variation
is up to 1 ms, i.e., up to 50% of the nominal 2 ms delay. In the context of busy-wait polling
loops this is inconsequential.

---

## Issue Summary

| ID | Severity | Location | Description |
|---|---|---|---|
| TM-01 | HIGH | `systick.h:52-62` | Systick ISR at 35–40% CPU load (700–800 μs / 2000 μs); thin headroom leaves little margin for any additional ISR work or load increase |
| TM-03 | MEDIUM | `adc.h:197-200`, `224-225` | ADC ISR uses slow `digitalWrite()` (~5 μs) for emitter control instead of `fast_write_pin()` (~60 ns); inconsistent with encoder ISR practice; adds ~20 μs unnecessary latency per cycle |
| TM-04 | MEDIUM | `mouse.h:395-413` | `Serial.print()` during active navigation adds 1–5 ms of blocking main-loop delay per cell; delays button-abort processing |
| TM-05 | LOW | `motors.h:222` | `MOTOR_MAX_PWM * v / battery_voltage` performs a float divide (~28 μs) in the systick ISR, twice per tick; could be replaced with cached reciprocal at minimal cost |
| TM-06 | LOW | `systick.h:65`; `sensors.h:197` | GPIO timing instrumentation present but commented out; no mechanism to re-enable for profiling without source edit |
| TM-07 | LOW | `encoders.h` (ISR_NOBLOCK interaction) | At max speed (2500 mm/s), encoder ISR preemption of systick adds up to ~120 μs jitter to systick completion; control law uses a fixed `LOOP_INTERVAL` constant so this appears as a small velocity/position estimation error |
| TM-08 | LOW | `reporting.h:228` | `report_radial_track()` outputs once per integer degree — irregular output rate during acceleration that cannot be correlated to fixed time intervals |
| TM-09 | LOW | — | No sub-millisecond time reference available; `millis()` cannot resolve intra-tick events; fine timing of ADC settle/stabilisation and ISR latency is not measurable without oscilloscope |

---

## Cross-cutting Observations

**The timing architecture is fundamentally sound.** The separation of concerns — hardware ISR
for control, main loop for navigation logic — is the correct approach for a resource-constrained
bare-metal system. The `ISR_NOBLOCK` choice for systick correctly prioritises encoder ISR latency
over systick execution atomicity, and the instruction in `systick.h:75` ("no code should follow
this line") correctly places the ADC trigger as the final act.

The primary tension is between **verbose run-time logging** (useful for development and post-run
analysis) and **main-loop responsiveness** during active navigation. The current logging volume
can delay button abort by tens of milliseconds over a full cell traversal. For competition use,
reducing or disabling `Serial` output during a maze run would improve abort responsiveness and
remove the longest unpredictable stall in the main-loop path.

The 40% CPU headroom consumed by the systick ISR at peak means the design has room for modest
additional complexity (e.g., a gyroscope update) but not for large additions without revisiting
the floating-point arithmetic budget.
