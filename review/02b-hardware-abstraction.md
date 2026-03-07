# Hardware Abstraction & Driver Architecture Review

## HAL Strategy

### What plays the role of a HAL

There is no dedicated HAL layer. Hardware access is organised into three tiers:

| Tier | Mechanism | Files |
|---|---|---|
| 1 — Arduino framework | `pinMode`, `digitalWrite`, `analogWrite`, `attachInterrupt`, `Serial`, `delay`, `millis` | Used throughout |
| 2 — AVR direct register access | Bit manipulation of `TCCR*`, `OCR2A`, `TIMSK2`, `ADCSRA`, `ADMUX` | `systick.h`, `adc.h`, `motors.h` |
| 3 — Direct port macros | `fast_write_pin` / `fast_read_pin` compile to single `PORT*` instructions | `config-ukmarsbot.h:87-88` |

The Arduino framework is used where speed is not critical (setup, serial I/O, LED output). It is deliberately bypassed for the three subsystems where latency or resolution matters: the ADC sequencer, the systick timer, and the motor PWM clock. Each bypass is justified and documented — see §HAL bypass justification below.

### Separation of hardware from application logic

The separation is **partial and pragmatic**, not architected:

- **Hardware pin constants** are centralized in `config-ukmarsbot.h` (all Arduino pin numbers) and per-robot config files (channel indices, polarity). No pin numbers appear in application code.
- **Peripheral drivers** (`AnalogueConverter`, `Encoders`, `Motors`, `Systick`) encapsulate their hardware access. Application code (`Mouse`, `CommandLineInterface`) never touches a register directly.
- **Sensor interpretation** and **motor control** are separated into distinct classes (`Sensors` vs `Motors`), though both ultimately depend on global singletons.
- **However**: the driver classes are not behind any interface or abstract base class. Swapping to a different MCU requires rewriting the driver class bodies directly — there is no HAL API to re-implement. `AnalogueConverter` in `adc.h:37` states explicitly: *"This class is specific to UKMARSBOT with a ATmega328p processor. If you are using any other processor, you will need to re-write this class."*

**Portability verdict**: The code can be ported to a different AVR variant by modifying the hardware config files. Porting to a different MCU family (STM32, RP2040, etc.) requires rewriting `adc.h`, `systick.h`, and the register-manipulation sections of `motors.h` and `config-ukmarsbot.h`. The application-layer classes (`Mouse`, `Motion`, `Maze`, `Profile`) are MCU-agnostic and could be reused unchanged.

### Direct register bypass — justification

| Register(s) | Location | Reason bypass is justified |
|---|---|---|
| `TCCR2A/B`, `TIMSK2`, `OCR2A` | `systick.h:27-35` | Arduino provides no API to configure Timer2 in CTC mode. `millis()` uses Timer0; `tone()` uses Timer2 — reclaiming Timer2 for a 500 Hz control loop is the standard pattern. |
| `TCCR1B` | `motors.h:288-300` | Arduino's default Timer1 PWM (pins 9/10) runs at 490 Hz, which is audible. Changing the prescaler to 1 gives 31.25 kHz, above the audible range. Arduino provides no API for this. |
| `ADCSRA`, `ADMUX` | `adc.h:127-131, 141-151, 178, 227` | Arduino's `analogRead()` is synchronous and blocking (~112 µs per channel). The dark/lit double-sampling scheme requires interrupt-driven sequencing across 8 channels — fundamentally impossible through the Arduino API. |
| `fast_write_pin` / `fast_read_pin` macros | `config-ukmarsbot.h:87-88` | Encoder and ADC ISRs must read/write GPIO in the minimum possible cycles. The Arduino `digitalRead`/`digitalWrite` functions involve ~50-ns table lookups and port register computation; the macros reduce this to a single machine instruction. |

---

## Peripheral Ownership & Initialization

### Initialization sequence (`mazerunner-core.ino:62-93`)

```
Serial.begin()           ← UART
pinMode(LEDs)            ← GPIO
adc.begin()              ← ADC peripheral + prescaler + emitter pins
motors.begin()           ← Motor GPIO + Timer1 PWM frequency
motors.disable_controllers()  ← safety: no motor output until systick is running
encoders.begin()         ← attach pin-change ISRs on INT0/INT1
systick.begin()          ← Timer2 CTC: starts ISR — delay(40) to stabilise
```

The sequencing is **correct**: systick is armed last, after all peripherals it drives are ready. The `delay(40)` after enabling Timer2 (20 full ticks) ensures the ADC conversion cycle has completed at least once before `loop()` begins.

The ADC sequencer does not start immediately when `adc.begin()` is called; `adc.start_conversion_cycle()` is called only from within `Systick::update()`. So the `ADC_vect` ISR cannot fire before systick starts. This is sound.

### Peripheral ownership

| Peripheral | Owner | Shared consumers | Protection |
|---|---|---|---|
| **ADC hardware** | `AnalogueConverter adc` | None — all reads go through `adc.get_raw/dark/lit()` | Interrupt-driven state machine; `get_raw()` uses `ATOMIC` |
| **Timer2 (CTC 500Hz)** | `Systick systick` | None | Sole owner after `begin()` |
| **Timer1 (PWM pins 9/10)** | `Motors motors` | Shared with Arduino framework's default PWM | Prescaler set once in `begin()`; framework `analogWrite` still drives the PWM duty |
| **INT0 / INT1 (pins 2/3)** | `Encoders encoders` | None | ISR callbacks registered via `attachInterrupt` |
| **UART** | `Serial` (Arduino) | `CommandLineInterface`, `Reporter` | Single object; no concurrent access — CLI runs in `loop()`, reporter in `mouse.*` methods called sequentially |

One ownership ambiguity: **Timer1** is configured for 31.25 kHz by `Motors::set_pwm_frequency()` (`motors.h:284-303`) but `analogWrite()` is still used to set duty cycle (`motors.h:253-268`). This works because `analogWrite` only writes the compare register, not the prescaler — but it is an implicit two-owner arrangement that is easy to break if someone calls `analogWrite` on a different pin driven by Timer1.

No peripheral is initialized more than once, and no conflicting configurations were found.

---

## Driver Quality

### Re-entrancy

**Systick ISR (`ISR_NOBLOCK`)**: The `ISR_NOBLOCK` attribute (`mazerunner-core.ino:57`) re-enables global interrupts immediately on entry. This allows encoder pin-change interrupts to preempt the systick body, preventing encoder count loss at high speed. It also means the ADC ISR can preempt systick.

The systick body calls (in order): `encoders.update()`, `motion.update()`, `sensors.update()`, `battery.update()`, `motors.update_controllers()`, `adc.start_conversion_cycle()`.

- `encoders.update()` uses `ATOMIC` to snapshot `m_left_counter`/`m_right_counter` (`encoders.h:142-147`). Correct — the encoder ISRs can preempt at any point.
- `sensors.update()` calls `adc.get_raw()` which uses `ATOMIC` (`adc.h:168-171`). Correct.
- `Profile::update()` does not use `ATOMIC` internally, but it is only called from systick and its shared state (`m_speed`, `m_position`) is only written by `update()`. Accessors from `loop()` context use `ATOMIC`. Correct.
- `Motors::update_controllers()` accesses `m_velocity`, `m_omega` which are also written by `motors.set_speeds()` (used nowhere in the current code) via `ATOMIC`. No race in practice.

**`adc.get_dark()` and `adc.get_lit()` — unguarded** (`adc.h:158-164`):

```cpp
int get_dark(const int i) const {
    return m_adc_dark[i];   // ← no ATOMIC
}
```

`m_adc_dark` is `volatile int[8]`. On AVR, `int` is 16 bits. Reading a 16-bit volatile is **not atomic** — the ADC ISR can update the high byte after the low byte is read, producing a torn value. This affects:
- `Battery::update()` (`battery.h:39`) — reads channel 7
- `Switches::update()` (`switches.h:52`) — reads channel 6 (also called from `loop()` to detect button presses)
- `Reporter::show_adc()` (`reporting.h:406`) — reads channels 6–7

**[MEDIUM]** In practice this is unlikely to cause observable problems (battery voltage and switch state change far slower than 500 Hz), but it is a latent data race. `get_raw()` correctly uses `ATOMIC`; `get_dark()` and `get_lit()` should too.

### `AnalogueConverter::do_conversion()` — no mutual exclusion guard

`adc.h:176-183`:
```cpp
/// Perform a 'manual' conversion of a channel
/// should not be used if the interrupt-driven sequencer is on
int do_conversion(uint8_t channel) {
    start_conversion(channel);
    while (ADCSRA & (1 << ADSC)) { }   // spin until done
    return get_adc_result();
}
```

The comment warns against concurrent use, but there is no runtime assertion, flag check, or critical section to enforce it. If called while the ADC ISR is active, both would write `ADMUX` and read `ADC` concurrently, corrupting the sequencer state machine.

**[MEDIUM]** `do_conversion()` is not called anywhere in the current codebase (confirmed by search), so the risk is latent. It exists as a utility for future use, and should either be removed or guarded.

### Infinite blocking waits — no timeout

`Profile::wait_until_finished()` (`profile.h:130-133`) and `Motion::wait_until_position()` (`motion.h:203-207`) are unconditional infinite loops:

```cpp
void wait_until_finished() {
    while (m_state != PS_FINISHED) {
        delay(2);
    }
}
```

If a motor stall, encoder failure, or floating-point edge case prevents the profile from reaching `PS_FINISHED`, the robot hangs forever with no recovery path, no LED indication, and no watchdog. The systick ISR continues to run (motors still driven), but all user interaction and sensor decisions stop.

**[MEDIUM]** There is no hardware watchdog configured anywhere in the codebase. A stuck `wait_until_finished()` call would require a manual reset.

### Floating-point in the ISR

`sensors.update()` is called from inside the 500 Hz systick ISR. It performs floating-point arithmetic for sensor scaling, cross-track error calculation, and the PD steering controller (`sensors.h:143-151`). The ATmega328P has no FPU — each `float` operation is a software-emulated multi-instruction sequence (~60–200 cycles each).

The code comments acknowledge this: *"Timing tests indicate that, with the robot at rest, the systick ISR consumes about 10% of the available system bandwidth … Two such active profiles increases it to about 35-40%"* (`systick.h:58-63`).

**[LOW]** The system works within budget by the author's measurements, but there is no headroom documented. Any significant addition to the ISR body risks overrunning the 2 ms tick budget.

### DMA

Not used. The ATmega328P has no DMA controller. Not applicable.

### Hardware error handling

No hardware error detection or recovery is implemented for any peripheral:

- **ADC**: No check for conversion timeout (the `ADSC` bit is polled in `do_conversion()` but not in the ISR path — if an ADC conversion fails to complete, the ISR state machine would hang in `LIT_READ` indefinitely).
- **UART**: Serial transmit/receive uses Arduino's buffered `Serial` object; no error flags are checked.
- **Motor/encoder**: No stall detection, no current limiting, no back-EMF check.
- **Battery undervoltage**: `battery.voltage()` is available but no code acts on it — there is no brownout protection beyond the AVR's hardware BOD fuse (not set in firmware).

**[LOW]** Appropriate for a contest robot where the operator is present and physical reset is always available. Would need addressing for an unattended or safety-critical deployment.

---

## Configuration Hygiene

### Pin assignments and hardware constants

Pin assignments are **well centralized**. All Arduino pin numbers for UKMARSBOT hardware appear only in `config-ukmarsbot.h`:

```
ENCODER_LEFT_CLK = 2, ENCODER_RIGHT_CLK = 3
ENCODER_LEFT_B = 4,   ENCODER_RIGHT_B = 5
USER_IO = 6
MOTOR_LEFT_DIR = 7,   MOTOR_RIGHT_DIR = 8
MOTOR_LEFT_PWM = 9,   MOTOR_RIGHT_PWM = 10
EMITTER_A = 11,       EMITTER_B = 12
```

ADC channel indices, sensor calibration constants, motor/encoder parameters, and turn parameters are in the per-robot config files (`config-robot-orion.h`, `config-robot-osmium.h`). No pin numbers or ADC channel numbers appear in application code.

Baud rate (`BAUDRATE = 115200`) is in the robot config file. The `platformio.ini` also specifies `monitor_speed = 115200` independently — these must be kept in sync manually. **[LOW]**

### Magic numbers

| Value | Location | Status |
|---|---|---|
| `OCR2A = 249` | `systick.h:34` | **[LOW]** Formula `(16000000/128/500)-1` is in the comment, not expressed as a named constant |
| `ADC prescaler bits: ADPS2=1, ADPS1=0, ADPS0=1` | `adc.h:127-129` | **[LOW]** Comment says "prescaler 32 → 500kHz" but the bit pattern is not mapped to a named constant |
| `ADMUX = DEFAULT << 6` | `adc.h:131` | **[LOW]** `DEFAULT` is an Arduino constant (= 1 for AVcc reference); the shift of 6 places the REFS bits correctly but the magic `6` is unexplained |
| `if (m_switches_adc > 800)` | `switches.h:63` | **[LOW]** Undocumented threshold. It detects button-fully-pressed (ADC near 1023). Should be `> BUTTON_THRESHOLD` or similar named constant |
| `m_sign * 5.0f` | `profile.h:215` | **[LOW]** The author comments *"magic number … I keep meaning to find a more tidy solution for"* — it is a minimum braking velocity to ensure the profiler crosses the finish point despite float rounding |
| `delay(40)` | `systick.h:36` | **[LOW]** 40 ms after enabling Timer2 equals 20 systick ticks; comment says "make sure it runs for a few cycles" — the number 40 should be derived from or named relative to `LOOP_INTERVAL` |
| `if (count > 5)` | `sensors.h:297, 304` | **[LOW]** Debounce count for hand-occlusion detection (5 × 20ms = 100ms). Named constant would clarify intent |

### Incorrect constant definition — operator precedence error

**[HIGH]** `config.h:36`:

```cpp
const float DEGREES_PER_RADIAN = 360.0 / 2 * PI;
```

Due to C++ left-to-right associativity of `*` and `/`, this evaluates as:

```
(360.0 / 2) * PI = 180 * π ≈ 565.49
```

The correct value is `360.0 / (2 * PI) = 180 / π ≈ 57.296`.

`DEGREES_PER_RADIAN` is **not referenced anywhere** in the codebase (confirmed by search) — the used constant is `RADIANS_PER_DEGREE = 2 * PI / 360.0` which is correct. However, if `DEGREES_PER_RADIAN` is ever used (e.g., when porting or extending the code), it will produce angle values approximately 9.87× too large, causing completely wrong motor outputs.

The paired correct constant (`RADIANS_PER_DEGREE`) is used at `motors.h:193` for the tangent speed calculation.

### Broken `Reporter::set_printer()` — reference semantics error

**[HIGH]** `reporting.h:98-110`:

```cpp
static Stream& printer = Serial;   // reference bound to Serial — permanent

void set_printer(Stream& stream) {
    printer = stream;              // copies stream INTO Serial, does not redirect
}
```

In C++, references cannot be rebound after initialization. `printer = stream` invokes `Stream::operator=()` (compiler-generated), which attempts to copy the internal state of `stream` over the already-bound `Serial` object. This does **not** redirect output — it corrupts or no-ops the assignment. The design intent (allow output to be redirected to a Bluetooth module or logger) is completely non-functional.

`set_printer` is called only once in the codebase (`mazerunner-core.ino:87`) with `Serial` itself as the argument, so in practice the self-assignment `Serial = Serial` is a harmless no-op. But the feature is broken. The fix is to use `static Stream* printer = &Serial;` and dereference it as `printer->print(...)`.

### Implicit Timer2 / `tone()` conflict

**[LOW]** `Systick::begin()` reconfigures Timer2 from the Arduino framework's default use (for `tone()` and `TimerTwo`). No comment guards this interaction. Calling `tone()` anywhere after `systick.begin()` would corrupt the 500 Hz control loop. Since `tone()` is not used, this is latent.

### `fast_write_pin` / `fast_read_pin` — silent portability regression

**[LOW]** `config-ukmarsbot.h:67-92`:

```cpp
#if defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
#define fast_write_pin(P, V)  BIT_WRITE(*__digitalPinToPortReg(P), ...)
#define fast_read_pin(P)      ...
#else
#define fast_write_pin(P, V)  digitalWrite(P, V)   // ← silently falls back
#define fast_read_pin(P)      digitalRead(P)
#endif
```

On an ATmega4809 (Arduino Nano Every) or any other AVR variant, both macros silently degrade to the slow Arduino API functions (~50× slower). The encoder ISR and ADC ISR would still function but at significantly higher CPU overhead, with no compile-time warning. A `#warning` or `#error` on the `#else` branch would make this visible.

---

## Summary Table

| # | Severity | Issue | Location |
|---|---|---|---|
| 1 | **HIGH** | `DEGREES_PER_RADIAN` computed as `180*π` (~565) instead of `180/π` (~57.3) due to operator precedence; unused but dangerous if referenced | `config.h:36` |
| 2 | **HIGH** | `Reporter::set_printer()` is non-functional; reference cannot be rebound; output is never redirected | `reporting.h:98-110` |
| 3 | **MEDIUM** | `adc.get_dark()` / `adc.get_lit()` have no `ATOMIC` guard; 16-bit reads on AVR are non-atomic — data race with `ADC_vect` | `adc.h:158-164` |
| 4 | **MEDIUM** | `AnalogueConverter::do_conversion()` has no runtime guard; concurrent use with interrupt-driven sequencer would corrupt ADC state machine | `adc.h:176-183` |
| 5 | **MEDIUM** | All motion blocking-waits are infinite loops; no timeout or watchdog means a stalled motor or bad encoder hangs the robot permanently | `profile.h:130`, `motion.h:203` |
| 6 | **LOW** | `OCR2A = 249`, ADC prescaler bits, `ADMUX` shift, switch threshold `800`, profile magic `5.0f`, debounce count `5` are unnamed literals | various |
| 7 | **LOW** | `BAUDRATE` in robot config and `monitor_speed` in `platformio.ini` must be kept in sync manually | `config-robot-orion.h:192`, `platformio.ini:7` |
| 8 | **LOW** | `fast_write_pin`/`fast_read_pin` silently fall back to slow `digitalWrite`/`digitalRead` on non-ATmega328P targets with no warning | `config-ukmarsbot.h:91-92` |
| 9 | **LOW** | Timer2 reuse conflicts with Arduino `tone()` — no comment guards against accidental use | `systick.h:27-35` |
| 10 | **LOW** | Floating-point PD steering controller runs in 500 Hz ISR; no FPU on AVR; timing budget is tight and undocumented beyond the original author's measurements | `sensors.h:143-151`, `systick.h:58-63` |
