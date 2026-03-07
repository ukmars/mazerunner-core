# Structural Survey: mazerunner-core

## 1. Project Identity

### Target Platform
- **MCU**: Atmel ATmega328P (Arduino Nano)
- **Clock**: 16 MHz
- **Architecture**: 8-bit AVR
- **RAM**: 2 KB SRAM; the design is explicitly RAM-conscious throughout (e.g. `PROGMEM` strings, 8-bit maze costs, stack-allocated queues, no heap use)

### Operating Environment
- **Bare metal**. No RTOS. No OS. No scheduler.
- Concurrency is managed manually via two hardware interrupts (Timer2 and ADC) plus two pin-change interrupts for the encoders. The main loop is a simple `loop()` polling function.
- Arduino framework is used (via PlatformIO), providing `setup()`/`loop()`, `delay()`, `analogWrite()`, `Serial`, etc.

### Primary Language and Standard
- **C++** (Arduino `.ino` entry point, all implementation in `.h` files — no `.cpp` files)
- Standard: not explicitly pinned in `platformio.ini`. PlatformIO's `atmelavr` + Arduino framework defaults to **GNU C++11** with AVR-libc.
- No explicit `-std=` flag is set in `build_flags`.

### Build System and Toolchain
- **PlatformIO** (`platformio.ini`) — the sole build system
- Toolchain: **avr-gcc** (GCC cross-compiler for AVR, installed and managed by PlatformIO automatically)
- Host: any machine running PlatformIO (Windows, Linux, macOS)
- Target: `nanoatmega328` (`platform = atmelavr`, `board = nanoatmega328`, `framework = arduino`)
- Additional build flags: `-Wl,-Map,firmware.map` (generate linker map), `-Wl,-u,vfprintf -lprintf_flt -lm` (enable float formatting in `printf`)
- Static analysis: `pio check` (cppcheck), with `-DCPPCHECK` defined when running
- Post-build: `post-build-script.py` runs after a successful build (purpose not examined — not source code)

### Cross-Compilation
- Cross-compilation is fully transparent through PlatformIO. Development happens on the host; the binary is flashed to the Nano over USB (`pio run --target upload`).
- No custom CMake, Makefile, or linker scripts are present; PlatformIO handles all of it.

---

## 2. Repository Structure

```
mazerunner-core-ukmars/
├── mazerunner-core/          ← ALL firmware source code (see §4)
│   ├── mazerunner-core.ino   ← Entry point: global objects + ISRs + setup() + loop()
│   ├── config.h              ← Top-level configuration (robot + event selection)
│   ├── config-ukmarsbot.h    ← Hardware pin assignments for UKMARSBOT v1.3A
│   ├── config-robot-orion.h  ← Robot-specific constants for ORION
│   ├── config-robot-osmium.h ← Robot-specific constants for OSMIUM (test robot)
│   └── *.h                   ← All subsystem implementations (header-only)
├── documents/
│   └── turn-designer.xlsx    ← Spreadsheet for calculating turn parameters
├── review/                   ← Code review outputs (this file)
├── platformio.ini            ← Build configuration
├── post-build-script.py      ← Post-build hook (runs after compile)
├── .clang-format             ← Code style (Google-based, 2-space indent, 160-col limit)
├── .travis.yml               ← CI configuration file (all content is commented out — CI not active)
├── doxyconfig                ← Doxygen configuration (documentation generation)
├── cppcheck.md               ← Notes on static analysis usage
├── include/README            ← PlatformIO placeholder (unused)
├── lib/README                ← PlatformIO placeholder (unused)
└── test/README               ← PlatformIO placeholder (no tests exist)
```

**This is a firmware-only repository.** There are no host-side tools, simulators, or test harnesses beyond the `pio check` static analysis. The `documents/` folder contains a design aid (Excel spreadsheet) but no generated documentation.

No git submodules. No vendored libraries (beyond what PlatformIO downloads). No package lock files.

---

## 3. Entry Points

### Main Entry Point
- **`mazerunner-core/mazerunner-core.ino`** — Arduino `setup()` and `loop()` are the entry points. On AVR with the Arduino framework the real `main()` is in the Arduino core library; it calls `setup()` once then calls `loop()` forever.

### ISR Registration

Two ISRs are defined explicitly in `mazerunner-core.ino`:

| ISR Vector | Registration | Purpose |
|---|---|---|
| `ADC_vect` | `ISR(ADC_vect)` at line 53 | ADC conversion complete — calls `adc.callback_adc_isr()` to advance the sensor sampling state machine |
| `TIMER2_COMPA_vect` | `ISR(TIMER2_COMPA_vect, ISR_NOBLOCK)` at line 57 | 500 Hz systick — calls `systick.update()` which runs the full control loop |

Two further ISRs are registered at runtime via the Arduino `attachInterrupt()` API inside `Encoders::begin()` (`encoders.h:56-57`):

| Pin | Trigger | Callback |
|---|---|---|
| `ENCODER_LEFT_CLK` (D2) | `CHANGE` | `callback_left_encoder_isr()` → `encoders.left_input_change()` |
| `ENCODER_RIGHT_CLK` (D3) | `CHANGE` | `callback_right_encoder_isr()` → `encoders.right_input_change()` |

`ISR_NOBLOCK` on `TIMER2_COMPA_vect` means encoder interrupts can preempt the systick ISR, preventing encoder count loss at high speed.

### Task/Thread Entry Points
There are none. This is single-threaded bare metal. All periodic work runs inside the Timer2 ISR (`systick.update()`). The `loop()` function does only: check for button press → dispatch function, and poll serial input.

---

## 4. Major Subsystems

All subsystems are implemented as classes in header-only files under `mazerunner-core/`. All instances are global singletons declared in `mazerunner-core.ino` and declared `extern` in their respective headers.

### 4.1 System Tick — `systick.h`
**Class**: `Systick`
**Purpose**: Configures Timer2 in CTC mode to fire at 500 Hz. The ISR body (`Systick::update()`) is the master control loop. Each tick it calls, in order: `encoders.update()`, `motion.update()`, `sensors.update()`, `battery.update()`, `motors.update_controllers(...)`, then `adc.start_conversion_cycle()`. The comment notes the ISR takes ~10% CPU at rest and ~35-40% with two active motion profiles.

### 4.2 ADC / Sensor Sampling — `adc.h`
**Class**: `AnalogueConverter`
**Purpose**: Interrupt-driven ADC sequencer. Samples all 8 channels twice per systick cycle — once with IR emitters off (ambient/"dark") and once with emitters on ("lit"). The difference `(lit - dark)` is the wall sensor reading. Channels 0–3 are wall sensors; channels 6–7 are battery and switches. The sequencer is a 5-state machine (`DARK_READ → EMITTER_ON → SETTLE → LIT_READ → COMPLETE`) driven by the ADC completion interrupt (`ADC_vect`). **ATmega328P-specific**: directly manipulates `ADCSRA`, `ADMUX`, `ADCSRA`.

### 4.3 Wall Sensors — `sensors.h`
**Class**: `Sensors`
**Purpose**: Converts raw ADC differences into normalised sensor values (`SensorChannel.value` = scaled to 100 at calibration position), sets boolean wall-detection flags (`see_left_wall`, `see_front_wall`, `see_right_wall`), and calculates a cross-track steering error. Runs a PD controller to produce a `m_steering_adjustment` (deg/s) that is fed into the motor controller. Called from systick. Steering mode is selectable: `STEER_NORMAL` (use whichever walls are visible), `STEER_LEFT_WALL`, `STEER_RIGHT_WALL`, or `STEERING_OFF`. Also provides `wait_for_user_start()` — blocks until a user occludes a front sensor by hand.

### 4.4 Wheel Encoders — `encoders.h`
**Class**: `Encoders`
**Purpose**: Quadrature encoder decoding via pin-change interrupts. Each interrupt fires on `CHANGE` of the clock pin; the second channel (B) is read in the ISR to determine direction. Uses `fast_read_pin()` macros (direct port register access) to minimise ISR latency. `update()` (called from systick) snapshots and resets the interrupt counters, then accumulates `m_robot_distance` (mm) and `m_robot_angle` (degrees). Provides per-tick change values (`robot_fwd_change()`, `robot_rot_change()`) consumed by the motor controller. `ATOMIC` blocks (AVR `ATOMIC_RESTORESTATE`) guard all multi-byte variable access.

### 4.5 Motion Profiles — `profile.h`
**Class**: `Profile`
**Instances**: `forward` (linear), `rotation` (angular) — both global, both updated by systick
**Purpose**: Trapezoidal speed profiler. Manages three phases: accelerate → coast → brake. `start()` configures a move; `update()` (called from systick) advances the profile by one tick, adjusting speed and accumulating position. Provides both non-blocking (`start()` + polling `is_finished()`) and blocking (`move()` = `start()` + `wait_until_finished()`) interfaces. `ATOMIC` blocks protect shared `volatile` state.

### 4.6 Motion Control — `motion.h`
**Class**: `Motion`
**Purpose**: High-level motion API used by the robot behaviour layer. Wraps `forward` and `rotation` profile instances. Provides named operations: `move()`, `spin_turn()`, `turn()`, `stop_at()`, `stop_after()`, `wait_until_position()`, `wait_until_distance()`, `adjust_forward_position()`. Also maintains cell-relative position through `set_position()` and `adjust_forward_position()` — the position counter is reset and offset at cell boundaries to keep floating-point values small.

### 4.7 Motor Drive — `motors.h`
**Class**: `Motors`
**Purpose**: Low-level DC motor control. Accepts forward velocity (mm/s) and angular velocity (deg/s) from the motion controller, plus a steering correction (deg/s). Runs two PD position controllers (forward and rotation error integration), adds velocity/acceleration feedforward, and splits the result into left/right motor voltages. Converts voltages to PWM via battery-compensated scaling (`pwm_compensated()`). Outputs via `analogWrite()` + direction pin. Sets Timer1 PWM to 31.25 kHz by directly writing `TCCR1B`. **ATmega328P-specific**: `TCCR1B`, `TCCR2A/B`, `TIMSK2`, `OCR2A`, `ADCSRA`, `ADMUX` are all accessed directly.

### 4.8 Battery Monitor — `battery.h`
**Class**: `Battery`
**Purpose**: Reads the battery voltage from an ADC channel (the "dark" reading of the unenergised channel, which is unaffected by IR emitter state). Applies a pre-calculated multiplier (`BATTERY_MULTIPLIER`) derived from the potential-divider resistor values. The current voltage is used by `Motors` to compensate PWM for battery droop.

### 4.9 Function Switches / Button — `switches.h`
**Class**: `Switches`
**Purpose**: Decodes a 4-bit DIP switch + pushbutton from a single ADC channel using a resistor ladder. `read()` returns 0–15 (switch combination) or 16 (button pressed). Provides `button_pressed()`, `wait_for_button_press/release/click()`. The ADC channel is shared with the main ADC sequencer; `Switches` reads directly from the already-converted dark value.

### 4.10 Maze Map — `maze.h`
**Classes**: `Maze`, `Location`, `Queue<T,N>`
**Purpose**: Stores a 16×16 micromouse maze. Each cell's four walls are packed into a `WallInfo` struct (4×2 bits = 1 byte per cell). Walls have four states: `EXIT`, `WALL`, `UNKNOWN`, `VIRTUAL`. A `MazeMask` (`MASK_OPEN` or `MASK_CLOSED`) controls how unknowns are treated during flood. The flood fill (`Maze::flood()`) is a BFS using a stack-allocated `Queue<Location, 64>`. `heading_to_smallest()` navigates from a cell toward the lowest-cost neighbour (the next step in the path). `update_wall_state()` only writes a wall once — once seen, it cannot be overwritten. The `Maze` object is placed in `.noinit` SRAM (via `PERSISTENT` macro) so it survives processor resets.

### 4.11 Mouse Behaviour — `mouse.h`
**Class**: `Mouse`
**Purpose**: The top-level behaviour layer. Implements all maze-solving and calibration routines. Key methods:
- `search_maze()` — search to goal and back, using flood-fill routing
- `search_to(target)` — flood-fill-guided search to any cell
- `follow_to(target)` — left-wall follower to a target cell
- `wander_to(target)` — random-direction wanderer
- `turn_smooth(turn_id)` — execute an SS90E turn triggered by distance or front sensor
- `turn_IP90L/R()`, `turn_IP180()` — in-place spin turns
- Calibration: `show_sensor_calibration()`, `conf_sensor_spin_calibrate()`, `conf_edge_detection()`, `conf_log_front_sensor()`, `test_SS90E()`

Turn parameters (`TurnParameters turn_params[4]`) are looked up by turn ID at runtime from a table defined in the robot config file.

### 4.12 Command Line Interface — `cli.h`
**Class**: `CommandLineInterface`
**Purpose**: Reads 32-character lines from `Serial`, tokenises on spaces/commas/`=`, dispatches single-character commands directly and multi-character commands via a PROGMEM command table. `run_function(n)` maps the hardware switch value (0–15) or `F n` serial command to robot behaviours.

### 4.13 Telemetry / Reporting — `reporting.h`
**Class**: `Reporter`
**Purpose**: Formatted serial output for debugging and calibration. Reports include: profile telemetry (position, speed, motor volts vs. time), sensor tracks (normalised and raw), radial spin data, maze wall display (plain / costs / directions), and per-cell action logging during search. Output target is configurable via `set_printer(Stream&)`; defaults to `Serial`.

---

## 5. Third-Party Dependencies

### Arduino Core (via PlatformIO `framework = arduino`)
- **Provides**: `setup()`/`loop()` wrapper, `Serial`, `delay()`, `analogWrite()`, `digitalWrite()`, `attachInterrupt()`, `millis()`, `pinMode()`, `constrain()`, `F()` macro, AVR-libc (`<util/atomic.h>`, `<wiring_private.h>`)
- **Version**: Managed by PlatformIO for the `atmelavr` platform; not pinned to a specific version in `platformio.ini`
- **Modification**: None. Used as-is.

### AVR-GCC / avr-libc (via PlatformIO toolchain)
- **Provides**: C runtime, `<stdint.h>`, `<math.h>`, `<util/atomic.h>`, `ATOMIC_BLOCK`
- **Version**: Managed by PlatformIO; not pinned.

### No other external libraries are declared in `platformio.ini`.

All code — including the queue, motion profiles, PD controllers, maze flood fill, and sensor processing — is written from scratch within this repository. There are no `lib_deps` entries.

**Notable absence**: The `lib/` and `include/` directories are empty PlatformIO scaffolding placeholders (each contains only a `README`). The `test/` directory similarly contains only a `README`; there are no unit tests.

---

## 6. Build Variants and Configuration

### Robot Selection (`config.h:119-128`)
Two robot variants are defined. The active one is selected by setting `ROBOT`:

| `#define ROBOT` value | Config file included | Robot name |
|---|---|---|
| `ROBOT_CORE_OSMIUM` (1) | `config-robot-osmium.h` | "OSMIUM CORE" — test robot, 20:1 gearbox, 6-magnet encoders |
| `ROBOT_ORION` (2) | `config-robot-orion.h` | "ORION" — contest robot, 11:1 gearbox, 18-magnet encoders |

**Current active robot**: `ROBOT_ORION` (`config.h:120`).

If neither is defined, a `#error "NO ROBOT DEFINED"` halts compilation.

### Hardware Platform Selection (`config.h:66-76`)
One hardware platform is currently supported:

| `#define HARDWARE` value | Config file included |
|---|---|
| `HARDWARE_UKMARSBOT_1_3A` (1) | `config-ukmarsbot.h` |

`config-ukmarsbot.h` defines all Arduino pin assignments and the `fast_write_pin`/`fast_read_pin` macros (direct PORT register access for ATmega328/328P). If `__AVR_ATmega328__` or `__AVR_ATmega328P__` is not defined, these fall back to `digitalWrite`/`digitalRead`. The `ATOMIC` macro similarly falls back to a no-op on non-AVR targets.

### Event / Venue Selection (`config.h:86-97`)
Sensor calibration constants vary by competition venue. Four events are defined; the active one is set via `EVENT`:

| `#define EVENT` value | Maze goal | Notes |
|---|---|---|
| `EVENT_HOME` (1) | `Location(1, 0)` | Home practice |
| `EVENT_UK` (2) | `Location(7, 7)` | UK contest |
| `EVENT_PORTUGAL` (3) | `Location(7, 7)` | Portugal contest |
| `EVENT_APEC` (4) | `Location(7, 7)` | APEC contest |

**Current active event**: `EVENT_HOME` (`config.h:93`). Each robot config file contains per-event blocks of calibration constants (`FRONT_LEFT_CALIBRATION`, `LEFT_CALIBRATION`, `TURN_THRESHOLD_SS90E`, etc.) selected by `#elif EVENT == ...`.

### Conditional Compilation Flags

| Flag | Where used | Effect |
|---|---|---|
| `CPPCHECK` (defined by `check_flags`) | guards in future/porting code | Allows static analysis to run without hardware headers |
| `__AVR_ATmega328__` / `__AVR_ATmega328P__` | `config-ukmarsbot.h:67` | Enables fast port-register I/O macros |
| `__AVR__` | `config-ukmarsbot.h:117` | Enables `ATOMIC_BLOCK` from `<util/atomic.h>` |
| `ARDUINO_ARCH_NRF52840` | `config-ukmarsbot.h:138` | Disables `redirectPrintf()` (incompatible with NRF52 compiler) |
| `DEBUG_LOGGING` | `config-robot-orion.h:196` (commented out) | Would enable profile data logging — currently disabled |

### Debug vs. Release
There is no explicit debug/release build distinction in `platformio.ini`. PlatformIO's default for Arduino targets compiles with optimisation (`-Os`) and without debug symbols. No `build_type` is set. Enabling `DEBUG_LOGGING` in the robot config is the primary mechanism for increasing diagnostic output; it is currently commented out.

### CI
A `.travis.yml` file exists but **all its content is commented out**. CI is not operational.
