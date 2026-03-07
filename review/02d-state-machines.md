# State Machines & Control Loops Review

---

## State Machine Inventory

### SM-1: Motion Profile — `Profile` class (`profile.h`)

**Two instances**: `forward` (linear mm) and `rotation` (angular deg), both global, both updated at 500 Hz from `Systick::update()`.

**Implementation**: explicit `enum State : uint8_t` + `if/else` chain in `update()`. The state variable is `volatile uint8_t m_state`.

**States and transitions**:

```
                  start(dist < 1mm)
                  stop() / finish()
                       ↓
PS_IDLE ──────────────────────────── PS_FINISHED
  ↑  start(dist ≥ 1mm)                ↑    ↑
  │       ↓                           │    │
  │  PS_ACCELERATING ─────────────────┘    │
  │       │  braking_distance ≥ remaining  │
  │       ↓                               │
  │  PS_BRAKING ──────────────────────────┘
  │                 remaining < 0.125 mm
  │
  └── reset() from any state
```

| State | Entry condition | What `update()` does |
|---|---|---|
| `PS_IDLE` | `reset()` | Returns immediately — no motor influence |
| `PS_ACCELERATING` | `start()` with valid distance | Ramps speed toward `m_target_speed`; checks braking distance each tick |
| `PS_BRAKING` | Braking distance threshold met | Ramps speed toward `m_final_speed` |
| `PS_FINISHED` | Distance threshold `< 0.125 mm` OR `stop()`/`finish()` | **Continues running** — still ramps speed toward target, still increments position |

**Critical detail**: `PS_FINISHED` is not a halted state. The update loop only returns early for `PS_IDLE`. In `PS_FINISHED`, speed continues to converge toward `m_target_speed` and `m_position` continues to increment. Motor drive is not suppressed until `reset_drive_system()` or `disable_drive()` is called externally. This is intentional (motors hold final speed) but creates a subtle implicit contract between `Profile` and its callers.

**No trap states**. All states have exit paths via `reset()`. `PS_FINISHED` is stable but not terminal from the motor perspective.

---

### SM-2: ADC Conversion Sequencer — `AnalogueConverter` (`adc.h`)

**Implementation**: `enum Phase { DARK_READ, EMITTER_ON, SETTLE, LIT_READ, COMPLETE }` + `switch` in `callback_adc_isr()`. Runs entirely within the `ADC_vect` ISR.

**Cycle** (one cycle per 500 Hz systick tick, triggered by `start_conversion_cycle()`):

```
start_conversion_cycle()
       ↓
  DARK_READ ─── (ch 0..7, one interrupt per channel) ───→ EMITTER_ON
                                                               ↓
                                                           SETTLE (one dummy read)
                                                               ↓
  COMPLETE ←── (ch 0..7) ─── LIT_READ                        ↓
      │                       ↑                           LIT_READ (starts ch 0)
      │                       └──────────────────────────────┘
      │
      └─── disables ADC interrupt, turns off emitters
           (waits for next start_conversion_cycle() from systick)
```

Each channel is sampled twice: once with emitters off (ambient = `m_adc_dark[]`), once with emitters on (reflected = `m_adc_lit[]`). The difference `get_raw()` = `lit - dark` is the wall sensor reading.

**DARK_READ internal channel counter**: The `m_channel` variable starts at 0 and increments on each interrupt. Transition to `EMITTER_ON` fires when `m_channel >= MAX_CHANNELS (8)`. The final dummy result of channel 8 (which doesn't exist) is read but discarded — an `analogRead(8)` on an ATmega328P wraps to channel 0 via the lower 3 bits of `ADMUX`. This is benign but architecturally imprecise.

**Default case** in the ISR switch (`adc.h:222`) routes to the COMPLETE action, providing safe behaviour for any corrupt `m_phase` value.

**`m_phase` initialises as `uint8_t m_phase = 0`** (`adc.h:240`), which coincidentally equals `DARK_READ = 0`. Safe, but brittle — if the enum order ever changes, the unstart state would be different.

---

### SM-3: Steering Mode — `Sensors` class (`sensors.h`)

**Implementation**: `uint8_t g_steering_mode` member + `if/else` chain in `sensors.update()`. No explicit enum-tagged state struct.

**States**:

| State | Value | Behaviour in `update()` |
|---|---|---|
| `STEER_NORMAL` | 0 | Uses whichever walls are visible (left, right, or both) |
| `STEER_LEFT_WALL` | 1 | Forces steering from left wall only |
| `STEER_RIGHT_WALL` | 2 | Forces steering from right wall only |
| `STEERING_OFF` | 3 | Cross-track error zeroed; steering output = 0 |

`set_steering_mode()` (`sensors.h:154-158`) resets `m_last_steering_error` and zeros `m_steering_adjustment` on every mode change. This correctly prevents a D-term spike when switching modes.

**Transitions**: entirely caller-controlled. The steering mode does not self-transition — it is set explicitly before and after every move, turn, and calibration routine. There are approximately 20 call sites in `mouse.h` alone.

**Pattern**: `STEERING_OFF` is always set before calling any `motion.*` function from a stopped state. `STEER_NORMAL` is re-enabled once the robot is moving and wall data is reliable. This pattern is consistent throughout but is an implicit convention, not enforced by any type system or assertion.

---

### SM-4: Mouse Behaviour State — `Mouse::State` enum (`mouse.h:39`)

**Declared but never used**:

```cpp
enum State { FRESH_START, SEARCHING, INPLACE_RUN, SMOOTH_RUN, FINISHED };
```

No member variable of type `State` exists in the `Mouse` class. No code reads or writes this enum. It is a skeleton for a planned state machine that was never implemented. The actual robot behaviour is controlled by direct function calls from `CommandLineInterface::run_function()`.

**[MEDIUM]** The absence of this state machine means there is no runtime guard against calling `search_maze()` when the robot is already moving, or calling `follow_to()` in an indeterminate pose.

---

### SM-5: Maze Flood Mask — `Maze` (`maze.h`)

**Implementation**: single `MazeMask m_mask` member, binary.

| Mask | Value | Effect on `is_exit()` |
|---|---|---|
| `MASK_OPEN` | `0x01` | Unknown walls treated as exits (used during search) |
| `MASK_CLOSED` | `0x03` | Unknown walls treated as walls (intended for fast runs) |

Default: `MASK_OPEN`. Initialise also forces `MASK_OPEN` (`maze.h:378`).

`MASK_CLOSED` is never set anywhere in the current codebase. The fast-run function `Mouse::run_to()` (`mouse.h:577-580`) is not implemented (`(void)target; //// Not implemented`). This means the mask's dual-mode purpose is entirely unused.

---

### SM-6: Implicit Main Loop Dispatch

**Implementation**: `loop()` in `mazerunner-core.ino:97-104` — no explicit state variable.

```
IDLE (polling loop())
  │
  ├── button pressed ──→ wait_for_button_release()
  │                          ↓
  │                      switches.read() → cli.run_function(n)
  │                          ↓ [blocking until function completes]
  │                      returns to IDLE
  │
  └── serial byte available → cli.process_serial_data()
         │
         └── complete line → execute_command() → run_function(n) [blocking]
                                               → returns to IDLE
```

During any `run_function()` call, the main loop is **entirely blocked**. Serial input received during a run is buffered by the 64-byte Arduino UART FIFO and processed only after the function returns. If the function runs for a long time (e.g., `search_maze()`), the FIFO fills and characters are silently dropped. There is no concurrent serial processing during a run.

---

### SM-7: `m_handStart` Flag — `Mouse` (`mouse.h:53, 921`)

A boolean flag with two logical states:
- `false` (default): robot is assumed at cell centre; `search_to()` backs up slightly before starting.
- `true`: robot is pressed against rear wall; `search_to()` moves directly forward.

Set to `true` at the start of each top-level run function (`search_maze`, `follow_to`, `wander_to`) and to `false` mid-run when returning from the goal. Not strictly a state machine, but it does affect the control flow of `search_to()` (`mouse.h:398-403`).

---

## Completeness & Correctness

### Profile SM

All four states have entry paths; all have exit paths via `reset()`. No unreachable states. No trap states (a state with no exit).

**Gap**: `PS_BRAKING` has no guard against `m_final_speed > m_target_speed`. The `start()` method clips `final_speed` to `top_speed` (`profile.h:89-91`), so this cannot be entered with a contradictory configuration. However, calling `set_target_speed()` externally during braking (`mouse.h:169`) can change `m_target_speed` mid-profile. No constraints prevent setting a target speed higher than the final speed during braking — the profiler would then accelerate while in `PS_BRAKING` state, which the state label implies should not happen. In practice, `set_target_speed()` is only called to freeze the current speed at turn-trigger time, so this does not cause problems currently.

**[LOW]** The `PS_BRAKING` → `PS_ACCELERATING` back-transition is impossible by design but there is no assertion preventing the state from being externally set via `set_state()` (`profile.h:136-138`). `set_state()` is public with no documentation of valid callers.

### ADC SM

The cycle is strictly linear. No back-transitions. The COMPLETE→DARK_READ restart requires external action (`start_conversion_cycle()`). If systick somehow fails to call `start_conversion_cycle()` (which cannot happen in the current architecture since it's the last line of `Systick::update()`), the ADC would stall in COMPLETE with stale sensor data. No timeout or watchdog detects this.

### Mouse::State enum

Completely unused — no completeness analysis possible. The lack of a mouse-level state machine is itself the issue.

### Startup / Reset initial state

| Object | Initial state | Safe? |
|---|---|---|
| `forward` / `rotation` profiles | `PS_IDLE` (`m_state = PS_IDLE`, `m_speed = 0`) | Yes |
| `Motors` | Controllers disabled (`disable_controllers()` called in `setup()`) | Yes |
| `Sensors` | `m_active = false`, `STEER_NORMAL` | Passive — sensors inactive, but g_steering_mode defaults to STEER_NORMAL |
| `Maze` | Random (`.noinit` SRAM, survives reset) | Conditionally safe — `initialise()` called only if button held during reset |
| `Mouse` | `m_handStart = false`, `m_heading = NORTH`, `m_location = (0,0)` | Yes — constructor calls `init()` |
| `ADC` sequencer | `m_phase = 0` = `DARK_READ`, emitters disabled | Yes — `begin()` called before systick |

**[MEDIUM]** `sensors.g_steering_mode` initialises to `STEER_NORMAL` (`sensors.h:106`). If code were ever called before `set_steering_mode(STEERING_OFF)` with a stationary robot, the steering controller would compute a cross-track error from uninitialised sensor data and inject it into the rotation controller. In practice, `Mouse::init()` calls `sensors.set_steering_mode(STEERING_OFF)` in its constructor, and `reset_drive_system()` is always called before enabling motors. But this dependency is implicit.

### Power loss / comms dropout

There is no communications link other than the Serial port. A USB disconnect or BT disconnect is invisible to the firmware — `Serial.print()` calls succeed (data goes to the UART TX register) but are never received. No state change occurs.

On power loss: no orderly shutdown is possible. The robot loses drive immediately when power drops. The maze is preserved in `.noinit` SRAM as long as some charge remains on the decoupling capacitors, but will be corrupted if power collapses slowly. There is no power-loss detection in the firmware.

---

## Control Loops

### CL-1: Forward Position Controller — `Motors::position_controller()` (`motors.h:102-108`)

**Type**: PD on accumulated position error (equivalent to a PID where the "I" term is the accumulated distance setpoint vs. encoder distance; no explicit integral gain — the accumulation is the plant model).

**Loop rate**: 500 Hz, called synchronously from `Systick::update()` via `motors.update_controllers()`.

**Equation** (per tick):
```
m_fwd_error += (velocity × LOOP_INTERVAL) - encoders.robot_fwd_change()
output = FWD_KP × m_fwd_error + FWD_KD × (m_fwd_error - m_previous_fwd_error)
```

**Gains** (ORION):
```
FWD_KP = 16 × FWD_TM / (FWD_KM × FWD_ZETA² × FWD_TD²)   ← derived analytically
FWD_KD = LOOP_FREQUENCY × (8 × FWD_TM - FWD_TD) / (FWD_KM × FWD_TD)
```
Derived at compile time from motor characterisation constants (`FWD_KM = 475 mm/s/V`, `FWD_TM = 0.190 s`, `FWD_ZETA = 0.707`). **Compile-time only; no runtime adjustment.**

**Output path**: summed with rotation output, passed to `set_left/right_motor_volts()`, clamped to `±MAX_MOTOR_VOLTS (6V)` (`motors.h:227, 234`), then battery-compensated to PWM.

**Integrator windup** (`m_fwd_error`): When motor output saturates (clamped at `±6V`), `m_fwd_error` continues to accumulate without limit. There is no anti-windup clamping. Mitigation: `reset_controllers()` explicitly zeroes `m_fwd_error` at the start of every motion sequence via `reset_drive_system()`. **[MEDIUM]** Within a single motion (e.g., a long straight), a sustained stall would cause `m_fwd_error` to grow at up to `SEARCH_SPEED × LOOP_INTERVAL = 0.8 mm/tick`. After 1 second, that is 400 mm of accumulated error. On obstacle removal, the controller would immediately saturate output at `+6V` until the error is paid down. No inter-sequence windup possible.

---

### CL-2: Rotation/Angle Controller — `Motors::angle_controller()` (`motors.h:124-130`)

**Type**: PD on accumulated rotational position error, with an additional external input (steering correction).

**Loop rate**: 500 Hz.

**Equation** (per tick):
```
m_rot_error += (omega × LOOP_INTERVAL) - encoders.robot_rot_change()
m_rot_error += steering_adjustment × LOOP_INTERVAL   ← from CL-3
output = ROT_KP × m_rot_error + ROT_KD × (m_rot_error - m_previous_rot_error)
```

**Gains** (ORION):
```
ROT_KP = 16 × ROT_TM / (ROT_KM × ROT_ZETA² × ROT_TD²)
ROT_KD = LOOP_FREQUENCY × (8 × ROT_TM - ROT_TD) / (ROT_KM × ROT_TD)
```
Same analytical derivation as CL-1, from `ROT_KM = 775 deg/s/V`, `ROT_TM = 0.210 s`. **Compile-time only.**

**Integrator windup**: Same situation as CL-1. `m_rot_error` is reset by `reset_controllers()`. During a turn, rotation output combines with forward output; total may saturate. No anti-windup.

**Steering injection**: `steering_adjustment × LOOP_INTERVAL` is added to `m_rot_error` each tick. This means the steering correction acts as a commanded angular velocity offset, integrated into the position error. The direction of the correction (positive = turn left) must match the sign convention of the controller. `STEERING_OFF` (mode 3) zeroes `steering_adjustment` before it reaches this equation, preventing noise injection during turns.

---

### CL-3: Wall Steering Controller — `Sensors::calculate_steering_adjustment()` (`sensors.h:143-151`)

**Type**: PD on cross-track error (lateral displacement from cell centreline, estimated from side sensor differentials). Output is an angular velocity correction (deg/s) injected into CL-2.

**Loop rate**: 500 Hz (called from `sensors.update()` inside systick, always — even when the output is zeroed by `STEERING_OFF` mode).

**Equation**:
```
pTerm = STEERING_KP × m_cross_track_error
dTerm = STEERING_KD × (m_cross_track_error - m_last_steering_error) × LOOP_FREQUENCY
adjustment = constrain(pTerm + dTerm, -STEERING_ADJUST_LIMIT, +STEERING_ADJUST_LIMIT)
```

**Output limit**: ±`STEERING_ADJUST_LIMIT` = ±90 deg/s (ORION). This caps the maximum steering correction. The constraint is applied before injection into CL-2.

**Gains** (ORION): `STEERING_KP = 6.3`, `STEERING_KD = 2.00`. **Compile-time only.**

**No integrator**: PD only — no windup possible in this controller. The D-term uses the previous error value; `set_steering_mode()` resets `m_last_steering_error` to `m_cross_track_error` to prevent a D-term spike when switching modes. This is correctly handled.

**Front wall suppression**: When `m_front_sum > FRONT_WALL_RELIABILITY_LIMIT`, the cross-track error is forced to zero (`sensors.h:254-256`). This prevents side sensor reflections from a front wall from driving spurious steering corrections. The suppression happens before the PD calculation, so both P and D terms go quiet.

**[LOW]** The D-gain `STEERING_KD = 2.00` is multiplied by `LOOP_FREQUENCY` in the equation (`sensors.h:147`): `dTerm = STEERING_KD × delta × LOOP_FREQUENCY`. In most PD controller descriptions, the D-term is `KD × (error - prev_error) / dt`. Multiplying by `LOOP_FREQUENCY` (= `1/dt`) is mathematically equivalent, but the gain value stored in the config (`2.00`) is therefore in units of `deg/s per (error_unit / tick)`, which is less intuitive than the more usual `deg/s per (error_unit / s)`. Worth noting when tuning.

---

### CL-4: Motor Feedforward — `Motors::leftFeedForward()` / `rightFeedForward()` (`motors.h:146-174`)

**Type**: Model-based open-loop feedforward (not a closed loop). Output is added directly to the PD controller output before clamping.

**Components** (per motor):
```
speedFF  = speed × SPEED_FF     ← proportional to velocity
biasFF   = ±BIAS_FF             ← static friction compensation
accFF    = ACC_FF × (speed - old_speed) × LOOP_FREQUENCY   ← acceleration (≈ d(speed)/dt)
```

The per-motor speeds are computed from `velocity ± omega × MOUSE_RADIUS × RADIANS_PER_DEGREE`.

**Constants** (ORION):
- `SPEED_FF = 1/FWD_KM = 1/475 ≈ 0.002105 V/(mm/s)`
- `ACC_FF = FWD_TM/FWD_KM = 0.190/475 ≈ 0.0004 V/(mm/s²)`
- `BIAS_FF = 0.340 V` (static friction offset)

The feedforward can be disabled by setting `m_feedforward_enabled = false` (`motors.h:341`). There is no setter — this flag can only be changed by directly accessing the private member. It defaults to `true` and is never changed at runtime in the current codebase.

**[LOW]** The feedforward uses separate `m_old_left_speed` / `m_old_right_speed` to differentiate speed, implicitly assuming it is called at exactly `LOOP_FREQUENCY`. If the call rate deviates (which it cannot in the current 500 Hz systick), the acceleration feedforward would be incorrect.

---

### CL-5: Trapezoidal Profile Generator — `Profile::update()` (`profile.h:201-243`)

**Type**: Not a closed-loop controller — this is an open-loop speed reference generator (trapezoidal velocity profile). It generates the `velocity` and `omega` setpoints consumed by CL-1 and CL-2 as their reference inputs.

**Loop rate**: 500 Hz.

**Operation**: ramps speed toward `m_target_speed` at `m_acceleration` each tick, transitions to braking when remaining distance ≤ braking distance.

---

## Control Loop Summary

| Controller | Type | Rate | Gains configurable? | Windup addressed? |
|---|---|---|---|---|
| CL-1 Forward position | PD (accumulated error) | 500 Hz ISR | Compile-time only | Reset between sequences; no clamp during saturation |
| CL-2 Rotation/angle | PD (accumulated error) | 500 Hz ISR | Compile-time only | Reset between sequences; no clamp during saturation |
| CL-3 Steering | PD (cross-track) | 500 Hz ISR | Compile-time only | N/A — no integral term |
| CL-4 Feedforward | Open-loop model | 500 Hz ISR | Compile-time only | N/A |
| CL-5 Profile | Open-loop generator | 500 Hz ISR | N/A | N/A |

**No runtime gain adjustment exists for any controller.** There is no CLI command, EEPROM storage, or runtime parameter update for any PD gain or feedforward constant. All must be recompiled. This is appropriate for a contest robot with a stable, well-characterised plant, but makes iterative tuning during a contest slow.

---

## Safety Interlocks

### Primary motor cut: `m_controller_output_enabled` flag

`Motors::update_controllers()` (`motors.h:202-205`) gates all motor voltage output behind this flag:

```cpp
if (m_controller_output_enabled) {
    set_right_motor_volts(right_output);
    set_left_motor_volts(left_output);
}
```

When `disable_controllers()` is called (`motors.h:53-55`), the PD math still runs (errors still accumulate) but no voltage reaches the motors. The motors are not actively braked — they coast to a stop. `motors.stop()` actively drives both motors to 0V before `disable_controllers()` is called via `reset_drive_system()`.

**This is the only software e-stop mechanism.** There is no hardware e-stop, no dedicated interrupt pin, no relay, and no watchdog timer configured in firmware.

### `motion.reset_drive_system()` — the safe-state entry point

Called at the start of every robot function and on exit. Sequence (`motion.h:54-62`):
1. `motors.stop()` → drive both motors to 0V immediately
2. `motors.disable_controllers()` → suppress future outputs
3. `encoders.reset()` → zero odometry
4. `forward.reset()` / `rotation.reset()` → profiles to PS_IDLE
5. `motors.reset_controllers()` → zero error accumulators
6. `motors.enable_controllers()` → re-enable output

Note: Step 6 re-enables the controllers immediately, while speed is still decelerating. The motors will be driven to maintain zero velocity target. This is intentional — active braking.

### Button abort during a run

`search_to()`, `follow_to()`, and `wander_to()` all check `switches.button_pressed()` at the top of their inner navigation loops (`mouse.h:280, 417, 521`):

```cpp
if (switches.button_pressed()) {
    break;
}
```

On break, each function calls `stop_at_center()` or `reset_drive_system()` to bring the robot to a controlled stop. **This is a graceful abort, not an emergency stop.**

**[HIGH]** The button abort cannot trigger during any blocking wait:
- `Profile::wait_until_finished()` — spins on `m_state != PS_FINISHED`
- `Motion::wait_until_position()` — spins on `forward.position() < position`
- `sensors.wait_for_user_start()` — spins on hand occlusion

If the robot stalls during a move, or if the user wants to abort during a `turn_smooth()`, `spin_turn()`, or `stop_at_center()` call (all of which block internally), the button has no effect. The robot continues driving the stalled motor at maximum clamped voltage until the profiler's distance target is reached (which it never will be), hanging forever.

### Output saturation — implicit interlock

Motor voltage is clamped at `±MAX_MOTOR_VOLTS = ±6V` (`motors.h:227, 234`) and PWM at `±255` (`motors.h:250, 261`). This prevents the controllers from commanding voltages beyond the hardware limit. The PWM is further battery-compensated: `pwm = MOTOR_MAX_PWM × desired_voltage / battery_voltage`. If battery voltage drops, PWM increases proportionally to maintain the desired voltage — up to the PWM limit. No undervoltage or overcurrent detection.

### Steering suppression during turns

`sensors.set_steering_mode(STEERING_OFF)` is called before every in-place turn and during profiled turns while the heading is changing (`mouse.h:91, 151, 195`). This prevents wall reflections during a turn from injecting spurious corrections into the rotation controller. The pattern is consistent and correctly applied in the existing function call sequences.

**[MEDIUM]** This is enforced by convention, not by the controller architecture. A future function that performs a turn without setting `STEERING_OFF` would silently activate wall steering during the rotation, likely causing the turn to overshoot.

---

## Issue Summary

| # | Severity | Issue | Location |
|---|---|---|---|
| 1 | **HIGH** | Button abort is ineffective during any blocking motion wait; a stalled robot cannot be stopped by the user until the blocking call returns (which it never will) | `profile.h:130`, `motion.h:203`, `mouse.h` throughout |
| 2 | **MEDIUM** | `Mouse::State` enum declared but never used — intended mouse-level state machine is absent; no guard against calling run functions in unsafe states | `mouse.h:39` |
| 3 | **MEDIUM** | No anti-windup clamping on `m_fwd_error` / `m_rot_error` during motor saturation; error accumulates freely during a stall within a motion sequence | `motors.h:104`, `motors.h:126` |
| 4 | **MEDIUM** | `sensors.g_steering_mode` initialises to `STEER_NORMAL` (not `STEERING_OFF`); startup is safe only because `Mouse::init()` always corrects it before motors are enabled — an implicit ordering dependency | `sensors.h:106`, `mouse.h:54` |
| 5 | **MEDIUM** | No runtime gain adjustment for any controller; all PD gains and feedforward constants require a recompile to change | `config-robot-orion.h`, `motors.h`, `sensors.h` |
| 6 | **LOW** | `Profile::set_state()` is public with no documentation of valid callers; external code could set `PS_BRAKING` from `PS_IDLE` or perform other nonsensical transitions | `profile.h:136` |
| 7 | **LOW** | `MASK_CLOSED` is never applied; `Mouse::run_to()` is unimplemented; the fast-run mode the maze mask was designed for does not exist | `mouse.h:577`, `maze.h:82` |
| 8 | **LOW** | `m_feedforward_enabled` has no public setter; it defaults to `true` and cannot be disabled at runtime for diagnostics | `motors.h:341` |
| 9 | **LOW** | Steering D-gain value in config (`STEERING_KD = 2.00`) is implicitly pre-divided by `dt` (multiplied by `LOOP_FREQUENCY` in the equation), making its units non-standard and the configured value unintuitive when tuning | `sensors.h:147`, `config-robot-orion.h:303` |
| 10 | **LOW** | ADC phase initialises as `uint8_t = 0` which only works correctly because `DARK_READ = 0` in the enum | `adc.h:240`, `adc.h:83` |
