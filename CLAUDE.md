# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build System

This is a PlatformIO project targeting an Arduino Nano (ATmega328). All source code is in `mazerunner-core/`.

```bash
# Build
pio run

# Build and upload to connected device
pio run --target upload

# Open serial monitor (115200 baud)
pio device monitor

# Run static analysis (cppcheck)
pio check
```

There are no automated tests — validation is done by running on the physical robot.

## Architecture

The `.ino` file (`mazerunner-core/mazerunner-core.ino`) defines all global singleton objects and ISRs. Everything is header-only — all class implementations are in `.h` files; there are no `.cpp` files.

**Global singletons** (declared in `.ino`, referenced via `extern` in headers):
- `systick` — 500Hz control loop timer (Timer2 ISR), calls `motors.update()` each tick
- `adc` — manages ADC conversions (ADC ISR); sensors read from it
- `encoders` — wheel encoder counts via Arduino pin-change callbacks
- `forward` / `rotation` — `Profile` instances for trapezoidal motion profiles
- `motors` — low-level PWM/voltage output with PD controllers
- `sensors` — wall sensor readings, steering logic, wall detection booleans
- `motion` — high-level move/turn commands built on `forward`/`rotation` profiles
- `maze` — 16×16 flood-fill maze map, stored in EEPROM (`.noinit` section survives reset)
- `mouse` — all robot behaviour: searching, wall following, calibration routines
- `cli` — serial command-line interface; also maps hardware function switches (0–15)
- `reporter` — formatted serial output for telemetry and maze display

**Control flow:**
1. Hardware button press → `switches.read()` → `cli.run_function(n)` → `mouse.*` method
2. Serial input → `cli.process_serial_data()` → `cli.execute_command()` → `mouse.*` method
3. Motion: `mouse` calls `motion.*` which drives `forward`/`rotation` profiles → `motors` via `systick` ISR

## Configuration

**Robot/event selection is done in `config.h`** by setting two `#define`s:
- `ROBOT` — selects the robot config file (e.g. `ROBOT_ORION` → `config-robot-orion.h`)
- `EVENT` — selects sensor calibration constants per venue (`EVENT_HOME`, `EVENT_UK`, `EVENT_PORTUGAL`, `EVENT_APEC`)

To add a new robot, create a `config-robot-<name>.h` and add it to the `#if` chain in `config.h`.

**Turn parameters** (`TurnParameters turn_params[4]`) are defined in the robot config file and cover SS90EL, SS90ER, SS90L, SS90R turns used during maze search.

## Key Behavioural Details

- **Maze persistence**: `Maze maze PERSISTENT` uses `__attribute__((section(".noinit")))` so the maze map survives processor resets. Hold the user button during reset to clear it.
- **Sensor start**: many functions wait for `sensors.wait_for_user_start()` which is triggered by briefly covering the front sensor with your hand.
- **Position tracking**: `motion.set_position()` and `motion.adjust_forward_position()` are used to maintain consistent cell-relative position tracking. The sensing position within each cell is `SENSING_POSITION` (170mm by default).
- **Steering modes**: set via `sensors.set_steering_mode()` — always set to `STEERING_OFF` when stationary or during turns.

## CLI Commands (Serial, 115200 baud)

| Command | Action |
|---------|--------|
| `?` / `HELP` | show help |
| `W` / `C` / `D` | display maze walls / costs / directions |
| `X` | reset maze |
| `B` | battery voltage |
| `S` | sensor readings |
| `E` | encoder readings |
| `F n` | run function n (same as setting hardware switches) |
| `SEARCH x y` | search to cell (x,y) |

## Code Review

A structured review has been completed. Documents are in `review/`:

| File | Contents |
|------|----------|
| `review/01-structural-survey.md` | Platform, repo layout, subsystem inventory |
| `review/02b-hardware-abstraction.md` | HAL strategy, peripheral ownership, driver quality |
| `review/02d-state-machines.md` | State machine inventory, control loops, safety interlocks |
| `review/03a-memory-safety.md` | Heap, stack, buffer handling, volatile/atomic analysis |
| `review/03b-error-handling.md` | Error propagation, fault response, retry/recovery, logging |
| `review/03c-timing.md` | ISR timing budget, jitter sources, blocking paths |
| `review/04-final-report.md` | Consolidated findings; Risk Register (FR-01–FR-32) |

GitHub issues were created for all 32 risk register items in `ukmars/mazerunner-core`:
- FR-01 = issue #16, FR-02 = #17, ..., FR-32 = #47

**To sync the risk register with closed GitHub issues**, ask:
> "sync the risk register with closed GitHub issues"

This will check `gh issue list --repo ukmars/mazerunner-core --state closed`,
cross-reference against FR-01–FR-32, and update `review/04-final-report.md`
to mark resolved items.

## CLI Commands (Serial, 115200 baud)

| Command | Action |
|---------|--------|
| `?` / `HELP` | show help |
| `W` / `C` / `D` | display maze walls / costs / directions |
| `X` | reset maze |
| `B` | battery voltage |
| `S` | sensor readings |
| `E` | encoder readings |
| `F n` | run function n (same as setting hardware switches) |
| `SEARCH x y` | search to cell (x,y) |

Function switch assignments (0 = safe/no-op):
- 1: sensor static calibration
- 2: search maze to goal and back
- 3: wall follow to goal
- 4: test SS90E turn
- 5: random wander
- 6: edge detection test
- 7: sensor spin calibration
- 8: front sensor distance table
- 9: move forward 4 cells
