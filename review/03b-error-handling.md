# Review: Error Handling Strategy

**Project:** mazerunner-core-ukmars
**Platform:** ATmega328P / Arduino Nano
**Date:** 2026-03-07
**Scope:** All source files in `mazerunner-core/`

---

## 1. Error Propagation Model

### Pattern in use

The project has no formal error propagation model. The dominant pattern is
**silent ignore**: when something goes wrong, most functions either do nothing,
return a fixed sentinel, or fall through silently. There are no return codes
used systematically, no errno, no exceptions (neither C++ nor custom), no
callbacks, and no out-parameters for error status.

| Function | Return type | Error representation |
|---|---|---|
| `Profile::wait_until_finished()` | `void` | None — infinite spin |
| `Maze::flood()` | `void` | None — queue overflow is silent |
| `Mouse::search_maze()` | `int` | Always returns `0` (see below) |
| `Switches::read()` | `int` | Returns `-1` on out-of-range ADC, but no caller checks it |
| `AnalogueConverter::start_conversion_cycle()` | `void` | Returns silently if `!m_configured` |
| `CommandLineInterface::handle_search_command()` | `void` | No argc check, no error feedback |

The sole use of a sentinel return value is `Switches::read()` returning `-1`
(`switches.h:73`), but no call site checks for that value; `button_pressed()`
calls `read() == 16` which simply treats `-1` as "not pressed".

### Consistency

The model is consistently absent rather than inconsistently present. Every
subsystem independently decides how to handle problems, but the decision is
uniformly the same: do nothing and continue. This simplifies the code but
leaves the system with no path from a detected abnormal condition to a
corrective action.

---

## 2. Unchecked Return Values

### Arduino/HAL calls

Arduino API functions called in `setup()` and `begin()` methods generally
have no return values (`pinMode`, `digitalWrite`, `analogWrite`, `Serial.begin`),
so there is nothing to check. This is a property of the Arduino API design, not
a code quality issue here.

**`attachInterrupt()`** (`encoders.h:56-57`) is the one Arduino call that can
silently fail: if the pin is not an interrupt-capable pin on the current board,
`attachInterrupt` does nothing and returns no error. The encoder ISRs would
never fire and the robot would drive blind. This would be immediately obvious
at run time but produces no diagnostic output.

### Internal API calls

**`Queue::add()`** (`maze.h:432`) — the BFS flood fill calls `queue.add(target)`
without checking for overflow. When the queue is full, `Queue::add()` silently
discards the item (`queue.h`, `return;` with comment). A silently truncated BFS
will produce incorrect cost values; the robot will navigate toward the wrong
cell. The comment in `maze.h` itself acknowledges the queue bound is "unproven".

**`motion.wait_until_position()`** (`motion.h:203`) — called without any check
on whether the profile was actually started or whether the current position
already exceeds the target. If `forward.position()` starts above `position`, the
call returns instantly rather than waiting; this is mostly benign but is not
documented as a precondition.

**`sensors.wait_for_user_start()`** (`sensors.h:286`) — the return value
(LEFT_START / RIGHT_START) is used in `conf_sensor_spin_calibrate()` and
`test_SS90E()` but ignored in `search_maze()`, `follow_to()`, `wander_to()`,
and `run()`. Those callers do not know or care which sensor was used to start.
This is not an error per se but means the choice of start sensor is silently
ignored in most run modes.

**`Serial.print()` / `Serial.println()`** — on AVR, these are blocking once
the 64-byte hardware UART TX buffer fills. There is no error return, but
excessive Serial output during a maze run can stall the `loop()` and delay
button-press detection (see §5).

---

## 3. Fault Response

### Hardware fault vectors

The ATmega328P has no Cortex-M style fault vectors (no HardFault, MemManage,
BusFault). Hardware-level faults it does have are:

- **Reset sources**: power-on reset, external reset (RESET pin), brown-out
  detection (BOD), watchdog timer reset.
- **BOD** is configured by fuse bits at flash time; the code neither reads nor
  sets fuse bits. Whether BOD is enabled depends on how the board was
  programmed. At default Arduino Nano fuse settings BOD is enabled at 2.7 V.
- No software resets are performed anywhere in the codebase.

### Watchdog timer

**There is no watchdog timer anywhere in the codebase.** The AVR watchdog
(`<avr/wdt.h>`, `wdt_enable()` / `wdt_reset()`) is never used. This is a
significant omission on a robot that can physically damage itself or its
environment if software wedges. The several infinite blocking loops (§4) are
the most likely wedge points:

- A stalled `Profile::wait_until_finished()` leaves motors running at
  whatever speed they were at before the hang.
- A stalled `stopAndAdjust()` front-wall loop leaves the robot pushing against
  a wall under full motor power.

Without a watchdog, the only recovery from such a wedge is a physical power
cycle or pressing the hardware RESET button.

### Defined safe state

There is no documented or enforced safe state. The closest analogue is the
`default:` case in `cli.run_function()` (`cli.h:482-486`):

```cpp
default:
    sensors.disable();
    motion.reset_drive_system();
    sensors.set_steering_mode(STEERING_OFF);
    break;
```

This disables sensors, stops and resets all controllers, and turns off
steering. It is appropriate but is only reachable via an unrecognised CLI
function number. There is no mechanism that calls it in response to an
observed fault.

`Mouse::panic()` (`mouse.h:708-714`) flashes LEDs and waits for a button press.
It is never called from any code path; it is dead code.

### Assert usage

No `assert()`, `static_assert()`, or custom assertion macros are used anywhere
in the project. `static_assert` would be valuable for compile-time checks of
configuration constants (e.g., confirming `MOTOR_MAX_PWM <= 255`,
`LOOP_FREQUENCY == 500`).

---

## 4. Retry & Recovery Logic

### Infinite blocking loops (potential wedge points)

The codebase contains numerous unconditional busy-wait loops that have no
timeout and no escape path other than a button press at the top-level
navigation loop.

| Location | Loop condition | Wedge scenario |
|---|---|---|
| `sensors.h:291` | `while (choice == NO_START)` | Sensor occlusion never detected — waits forever before every run |
| `switches.h:81` | `while (not button_pressed())` | Button circuit open — waits forever |
| `motion.h:152` | `while (forward.speed() != 0)` | Float equality — likely safe but formally undefined termination |
| `mouse.h:96-109` | `while (sensors.get_front_sum() < FRONT_REFERENCE)` | Sensor loses front wall — pushes forever |
| `mouse.h:200-207` | `while (sensors.get_front_sum() < FRONT_REFERENCE)` | Same as above |
| `reporting.h:399` | `while (true)` | Unconditional infinite loop in `show_adc()` |

The `show_adc()` function (`reporting.h:399-413`) is particularly notable:

```cpp
void show_adc() {
    while (true) {          // no exit
        ...
        delay(50);
    }
    sensors.disable();      // UNREACHABLE CODE
}
```

`sensors.disable()` after the `while (true)` is dead code; the emitters are
never turned off if `show_adc()` is called. A power cycle is required to exit.

### BLOCKED heading — silent misdirection

**HIGH** `Maze::heading_to_smallest()` returns `BLOCKED` (value 99) when no
exit is found — for example when the maze is improperly flooded or the robot
is truly walled in. The caller in `search_to()` (`mouse.h:426-427`) does:

```cpp
unsigned char newHeading = maze.heading_to_smallest(m_location, m_heading);
unsigned char hdgChange = (newHeading - m_heading) & 0x3;
```

When `newHeading` is `BLOCKED` (99), the subtraction wraps modulo 4. For a
robot heading NORTH (0): `(99 - 0) & 0x3 == 3 == LEFT`. The robot executes a
left turn instead of stopping or reporting an error. It will then make another
left turn, and another, spinning indefinitely in a 4-cell square while emitting
no diagnostic output. This is the closest the project comes to an unbounded
retry loop and it is invisible to the operator.

### Error escalation

There is no error escalation path of any kind. Individual subsystems do not
signal errors to their callers. Callers do not signal errors upward. There is
no concept of a system fault level. The only system-level "escalation" is the
button check at the top of navigation loops and inside blocking waits via
`motion.emergency_stop()`.

---

## 5. Logging & Diagnostics

### Logging system

There is no dedicated logging subsystem. All diagnostic output goes directly
to `Serial` via `Serial.println()` / `printer.print()` call sites scattered
through `mouse.h`, `cli.h`, and `reporting.h`. The `Reporter` class provides
formatted reports but not a log with severity levels, timestamps, or routing
control.

The `set_printer()` method in `reporter.h:108-110` was intended to allow
routing to a different `Stream` (e.g., a Bluetooth serial port), but as noted
in the memory safety review it is broken: the `static Stream& printer`
reference cannot be rebound, so all output always goes to `Serial`.

### Timing impact

**MEDIUM** `Serial.println()` and `Serial.print()` on AVR are blocking once
the 64-byte hardware TX buffer fills. At 115200 baud, draining 64 bytes takes
approximately 5.6 ms. During a maze run, `search_to()` emits several lines of
text per cell via `reporter.log_action_status()` (`mouse.h:421`) and
`update_map()` (`mouse.h:616`). If the host is slow to read, the TX buffer
fills and the 500 Hz control loop is blocked inside `Serial.print`. This does
not hang the ISR (which is interrupt-driven) but delays the main-loop button
check and reduces the responsiveness of command input during a run.

### Error context

When things go wrong, the logs provide:

- Cell location and heading at each navigation step (`log_action_status`).
- Front sensor sum and robot position at turn trigger points.
- Action character (F/L/R/B) and trigger source (sensor 's' vs. distance 'd').

What is absent:

- **No severity levels.** Informational and diagnostic output are
  indistinguishable.
- **No timestamps.** It is not possible to reconstruct when during a run an
  event occurred from the log alone. The profile reporter (`report_profile()`)
  does include a relative timestamp, but navigation logging does not.
- **No fault log.** Anomalous conditions (BLOCKED heading, queue overflow, ADC
  not configured) are silently swallowed with no output.

### Persistent fault logs

There is no persistent fault storage. The `Maze` object is placed in `.noinit`
via `PERSISTENT` and survives warm resets, but no fault history is preserved
alongside it. If the robot crashes and is reset, all run-time state is lost.

---

## Issue Summary

| ID | Severity | Location | Description |
|---|---|---|---|
| EH-01 | HIGH | `mouse.h:426-427` | `BLOCKED` heading (99) aliased to `LEFT` turn; walled-in robot spins silently forever |
| EH-02 | HIGH | — | No watchdog timer; hanging blocking loops leave motors running with no automatic recovery |
| EH-03 | MEDIUM | `reporting.h:399` | `while(true)` in `show_adc()` — requires power cycle to exit; `sensors.disable()` unreachable |
| EH-04 | MEDIUM | `mouse.h:96-109`, `200-207` | Front-wall approach loops have no timeout and no button escape; push against wall until power cut |
| EH-05 | MEDIUM | `maze.h:432` | `queue.add()` overflow silently discards BFS frontier cells; incorrect flood result, wrong navigation |
| EH-06 | MEDIUM | `mouse.h:708-714` | `panic()` is defined but never called; no code path leads to it |
| EH-08 | LOW | `encoders.h:56-57` | `attachInterrupt()` silent failure on wrong pin not detectable at runtime |
| EH-09 | LOW | `switches.h:73` | `Switches::read()` returns `-1` on invalid ADC range; no call site checks it |
| EH-10 | LOW | — | No `static_assert` guards on configuration constants; misconfiguration produces no compile-time diagnostic |
| EH-12 | LOW | `reporting.h:108-110` | `set_printer()` is a broken no-op; no diagnostic if a caller assumes output was redirected |

---

## Cross-cutting Observations

**The project trades reliability for simplicity in a way that is appropriate
for its hardware constraints and competitive context.** An ATmega328P with 2 KB
SRAM cannot host a rich logging or fault management subsystem. The primary
safety mechanism is the physical button abort, which works well for most
routine scenarios.

The highest-risk gap is the combination of **no watchdog** and **unescapable
inner loops**: a robot running into an unexpected sensor condition (EH-04) or a
corrupted maze state (EH-01) can become physically stuck with motors powered
and no automatic path to recovery. On a 700g robot with rubber wheels this is
unlikely to cause damage, but adding a watchdog with a period of ~500 ms would
provide a last-resort reset at minimal code cost (~10 bytes).
