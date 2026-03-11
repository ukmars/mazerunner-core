# Memory Safety Audit

Platform context for all findings: **ATmega328P, 2 KB SRAM, 32 KB Flash, 8-bit, no MMU, no hardware stack guard.**

---

## Heap Usage

**No dynamic allocation is used anywhere in the firmware.** A search for `malloc`, `free`, `new`, `delete`, `realloc`, and `calloc` finds zero hits in the `mazerunner-core/` source.

The Arduino `HardwareSerial` object (`Serial`) internally uses fixed-size 64-byte ring buffers allocated statically by the framework — not from a runtime heap. These are allocated once and never freed.

No custom allocator or memory pool is present. All storage is either:
- **Static / global**: global singleton objects in `.bss` / `.data` / `.noinit`
- **Stack**: local variables and function arguments
- **Flash (PROGMEM)**: string literals via `F()` macro and `PSTR()`

**Heap fragmentation risk: none.** The absence of dynamic allocation eliminates this entire class of problem.

---

## Stack Depth

### Platform constraints

The ATmega328P has a single unified 2 KB SRAM region shared between the BSS segment (global objects), the stack (grows down from top of SRAM), and any heap (not used here). There is **no hardware stack guard, no canary, and no MPU**. Stack overflow silently corrupts adjacent data — typically the `.bss` segment containing global singletons — with no detectable fault.

### No recursion

No recursive functions exist in the codebase. The flood fill (`Maze::flood()`, `maze.h:417`) is iterative BFS using an explicit queue. The search loop (`Mouse::search_to()`, `mouse.h:392`) is iterative with a `while` loop. No recursion risk.

### Significant stack allocations

**`Maze::flood()` — `maze.h:430`:**
```cpp
Queue<Location, MAZE_CELL_COUNT / 4> queue;  // MAZE_CELL_COUNT/4 = 64
```
`Queue<Location, 64>` layout:
- `mData[num_items + 1]` = `Location[65]` = 65 × 2 bytes = **130 bytes**
- `mHead`, `mTail`, `mItemCount` (3 × `int`) = 6 bytes
- Total: **136 bytes** on the stack

This is the single largest stack allocation in the codebase. `flood()` is called from:
- `Mouse::search_to()` at the start of each navigation move (`mouse.h:393, 425`)
- `Mouse::search_maze()` (`mouse.h:673, 674`)
- `Reporter::print_maze()` for display (`reporting.h:463`)
- `Maze::initialise()` (`maze.h:379`)

At the point of call, the stack already contains frames for `loop()` → `cli.run_function()` → `mouse.*()` → `search_to()` → `flood()`. Each frame includes saved registers and local variables. On AVR with the Arduino framework, function call overhead is typically 10–30 bytes per frame. The total call depth for `flood()` reaches 5–6 frames, so the stack is consuming roughly 200–300 bytes at this point, with 136 bytes for the queue on top. In a 2 KB system, this is **a significant fraction of available stack space**.

**[MEDIUM]** No static stack analysis or worst-case stack depth measurement has been performed. The `pio check` tool (cppcheck) does not perform stack depth analysis. Compiling with `-fstack-usage` and summing frame sizes along the deepest call path would confirm whether a margin exists.

**CLI buffer allocations — `cli.h`:**
- `Args args` stack-allocated in `process_serial_data()` (`cli.h:207`) and `process_input_line()` (`cli.h:291`): `char *argv[16]` + `int argc` = 16 × 2 + 2 = **34 bytes** (pointers are 2 bytes on AVR)
- `char buf[60]` in `handle_calibrate_encoders_command()` (`cli.h:428`): **60 bytes**
- `char buf[20]` in `handle_search_command()` (`cli.h:356`): **20 bytes** — see buffer overflow finding below
- `char buffer[16]` in `get_command_index()` (`cli.h:319`): 16 bytes

**ISR stack usage:**

The systick ISR body (`Systick::update()`, `systick.h:64-76`) calls `encoders.update()`, `motion.update()`, `sensors.update()`, `battery.update()`, `motors.update_controllers()`, and `adc.start_conversion_cycle()`. Each requires saving and restoring registers. On AVR, a complex ISR with floating-point software routines can consume 50–100 bytes of stack for register saves alone.

`ISR_NOBLOCK` on the Timer2 ISR means the encoder pin-change ISRs can preempt it, stacking a second ISR frame on top. The encoder ISRs are simple (read pins, arithmetic) but still require register saves (~20–30 bytes).

**Worst-case ISR stack depth** (not measured): systick ISR frame + encoder ISR preemption frame ≈ 120–200 bytes of stack used by ISRs concurrently, on top of whatever the main call stack holds at the time of preemption.

**No stack overflow detection is enabled or available on this platform.** There is no equivalent of FreeRTOS `uxTaskGetStackHighWaterMark()` or ARM Cortex-M stack monitor. The only detection would be post-hoc observation of corrupted global variables.

---

## Buffer Handling

### [HIGH] `args.argv[N]` dereferenced without checking `args.argc`

**Location 1**: `cli.h:349, 353` — `handle_search_command()`:
```cpp
void handle_search_command(const Args &args) {
    int x = 0;
    int y = 0;
    if (!read_integer(args.argv[1], x)) { ... }  // args.argc may be 1
    if (!read_integer(args.argv[2], y)) { ... }  // args.argc may be 1 or 2
```

**Location 2**: `cli.h:403` — `run_short_cmd()`, case `'F'`:
```cpp
case 'F': {
    int function = -1;
    int digits = read_integer(args.argv[1], function);  // args.argc may be 1
```

`Args` is stack-allocated without explicit initialization (`cli.h:207`). The `argv` array contains 16 `char*` pointers; only those filled by `tokenise()` are valid. If `args.argc == 1`, then `args.argv[1]` through `args.argv[15]` contain whatever uninitialized values were on the stack. Reading them as `char*` pointers and then dereferencing via `read_integer()` or `read_float()` is **undefined behaviour** — it reads from an uninitialized pointer.

On AVR with a flat 16-bit address space, the garbage pointer value will typically point somewhere in SRAM (e.g., to another local variable, to a global, or to a hardware register). `read_integer` scans the pointed-to memory byte-by-byte for ASCII digits (`'0'`–`'9'`, values 0x30–0x39). Most garbage memory content will not match, so the function returns 0 and the caller uses the default value. The program does not usually crash. However:
- Reading from hardware I/O register space (addresses 0x0000–0x001F on ATmega328P map to CPU registers, 0x0020–0x00FF map to I/O) can have side effects.
- On a different platform (if the code is ever ported) this would be a clean crash or security hole.

**Fix**: guard with `if (args.argc > 1)` before accessing `args.argv[1]`, etc.

### [LOW] `buf[60]` size in `handle_calibrate_encoders_command()` — `cli.h:428-429`

```cpp
char buf[60];
sprintf_P(buf, PSTR("L:%4d R:%4d P:%5.2f A:%5.2f\r\n"),
          encoders.m_total_left, encoders.m_total_right, position, angle);
```

`%4d` with a 16-bit int can produce up to 6 chars (`-32768`). `%5.2f` with a 32-bit float can theoretically produce very long strings if `position` or `angle` accumulates to a large value (no reset guard). For realistic encoder usage (short calibration runs), the output is well within 60 bytes. However, if `m_total_left` or `m_total_right` overflow (they are `int`, no range limit — noted in `encoders.h:236`), or if position accumulates, the buffer could overflow. Rated LOW given the operational context, but the use of `sprintf` rather than `snprintf` is the root cause.

### CLI input buffer — correctly bounded

`CommandLineInterface::add_to_buffer()` (`cli.h:231-236`) correctly gates on `m_index < INPUT_BUFFER_SIZE - 1` before writing. `m_buffer` is declared as `char m_buffer[INPUT_BUFFER_SIZE]` = 32 bytes (`cli.h:537`). `m_index` is `uint8_t` (`cli.h:538`). The buffer cannot overflow via `add_to_buffer`. Backspace handling correctly checks `m_index > 0` before decrementing. **Safe.**

### `strncpy_P` in `get_command_index()` — correctly bounded

`cli.h:321`:
```cpp
strncpy_P(buffer, (char *)pgm_read_word(&(commands[i])), sizeof(buffer) - 1);
```
`buffer` is `char buffer[16]`. The `strncpy_P` call uses `sizeof(buffer) - 1 = 15` as the length limit. The longest command string in `commands[]` is `"SEARCH"` (6 chars + NUL). **Safe.** Note: `strncpy_P` does not guarantee NUL termination if the source is longer than the limit, but the immediately following `strcmp` would then read past the buffer. In practice this cannot happen since all command strings are shorter than 15 characters, but a `buffer[15] = '\0'` guard after the copy would be defensive.

### `strtok` modifying the input buffer — documented, safe within scope

`cli.h:257` uses `strtok(line, " ,=")` which modifies the buffer in-place, replacing delimiters with `'\0'`. The `argv` pointers in `Args` point into `m_buffer`. After `tokenise()`, these pointers remain valid as long as `m_buffer` is not cleared or the function is not re-entered. `clear_input_buffer()` is called after `execute_command()` returns (`cli.h:211`), by which time all `argv` access is complete. **Safe within current usage.**

### Maze array indexing — wrapped, not bounds-checked

All maze array accesses use `m_walls[cell.x][cell.y]` and `m_cost[cell.x][cell.y]` where `cell` is a `Location`. `Location::north()`, `south()`, `east()`, `west()` all use modular arithmetic to wrap coordinates:

```cpp
// maze.h:183-184
Location north() const {
    return Location(x, (y + 1) % MAZE_HEIGHT);  // wraps 15 → 0
}
```

This ensures array indices are always in `[0, MAZE_WIDTH)` / `[0, MAZE_HEIGHT)` — **no out-of-bounds array access is possible via these accessors**. However, wrapping rather than clamping or erroring means the robot's logical position silently wraps to the opposite edge if it navigates past the boundary. In a correctly walled maze this cannot occur; in a corrupted maze map it could.

`Location::is_in_maze()` (`maze.h:169-171`) exists but is never called in the codebase. It is therefore not used as a guard.

### Queue overflow — silent data loss — `queue.h:55-57`

```cpp
void add(item_t item) {
    if (mItemCount >= num_items) {
        return;  // but drop the item for now
    }
    ...
}
```

When the queue is full, new items are silently discarded. In `Maze::flood()`, the queue holds the BFS frontier. If a frontier cell is dropped, it and all cells reachable only through it receive cost `MAX_COST` (unreachable) rather than their true BFS distance. The robot's path planning would then route around those cells or consider the maze unsolvable.

The queue is sized at `MAZE_CELL_COUNT / 4 = 64` Locations. The comment (`maze.h:429-430`) acknowledges this sizing is unproven:

> *"I believe the maximum size that this queue can possibly be for a classic maze is 64 (MAZE_CELL_COUNT/4) cells. HOWEVER, this is unproven"*

**Worst-case BFS frontier analysis**: In an empty 16×16 grid (no internal walls), a BFS from a corner expands in a diamond pattern. The frontier at step `k` contains all cells at Manhattan distance `k` from the origin. The maximum frontier size occurs at `k = 15` (centre of the grid) and contains up to 31 cells for a grid BFS. However, for a more general graph BFS, the frontier can be larger. For a fully connected rectangular grid, the peak frontier size is approximately `N/4` where N is the number of reachable cells. For 256 cells that is 64 — exactly the queue size. This is empirically tight but may be sufficient for standard contest mazes which have mandatory walls reducing connectivity.

**[MEDIUM]** The risk is not a crash but silent wrong navigation. Increasing the queue to 128 elements would cost 4 bytes more per `Location` × 64 extra = 128 bytes of stack, which is significant on this platform.

---

## Volatile & Memory Ordering

### Single-core architecture: no memory barriers required

The ATmega328P is a single-core processor. There are no cache coherency issues, no write buffers, and no out-of-order execution. The C++ `volatile` keyword, combined with `ATOMIC_BLOCK(ATOMIC_RESTORESTATE)` for multi-byte quantities, is sufficient for all ISR-shared data. No explicit memory barriers (e.g., `__sync_synchronize()`, `std::atomic_thread_fence()`) are needed or relevant.

### Correctly volatile: ISR-written, main-read variables

| Variable | Location | ISR writer | Main reader | Assessment |
|---|---|---|---|---|
| `m_adc_dark[8]` | `adc.h:233` | `ADC_vect` | `battery.h`, `switches.h` | `volatile int` — correct; `get_dark()` lacks ATOMIC (see below) |
| `m_adc_lit[8]` | `adc.h:234` | `ADC_vect` | `sensors.h` | `volatile int` — correct |
| `Profile::m_state` | `profile.h:246` | Timer2 ISR | `loop()` | `volatile uint8_t` — correct; single-byte atomic on AVR |
| `Profile::m_speed` | `profile.h:247` | Timer2 ISR | Motion callers | `volatile float` — needs ATOMIC for 4-byte reads; consistently guarded |
| `Profile::m_position` | `profile.h:248` | Timer2 ISR | Motion callers | `volatile float` — same |
| `Encoders::m_robot_distance` | `encoders.h:226` | Timer2 ISR | `loop()` | `volatile float` — correct; `robot_distance()` uses ATOMIC |
| `Encoders::m_robot_angle` | `encoders.h:227` | Timer2 ISR | `loop()` | Same |
| `Sensors::see_left/front/right_wall` | `sensors.h:100-102` | Timer2 ISR | `mouse.h` navigation loop | `volatile bool` — 1 byte, atomic on AVR — correct |
| `Sensors::m_cross_track_error` | `sensors.h:323` | Timer2 ISR | `reporting.h` | `volatile float` — 4 bytes, no ATOMIC guard on reads — see below |
| `Sensors::lfs/rss/lss/rfs` | `sensors.h:95-98` | Timer2 ISR | `reporting.h`, `mouse.h` | `volatile SensorChannel` — struct, multi-field reads non-atomic — see below |

### [MEDIUM] `volatile SensorChannel` struct — non-atomic multi-field reads

```cpp
// sensors.h:83-86
struct SensorChannel {
    int raw;    // 2 bytes
    int value;  // 2 bytes
};
volatile SensorChannel lfs;  // sensors.h:95
```

`sensors.update()` writes `lfs.raw` then `lfs.value` as two separate volatile stores:
```cpp
// sensors.h:211, 218
lfs.raw = adc.get_raw(LFS_ADC_CHANNEL);
// ... other channels ...
lfs.value = FRONT_LEFT_SCALE * lfs.raw;
```

Code in `reporting.h` and `mouse.h` reads both fields:
```cpp
// reporting.h:205, 209
print_justified(sensors.lfs.raw, 6);
// ...
print_justified(sensors.lfs.value, 6);
```

Between reading `lfs.raw` and reading `lfs.value` from main context, the systick ISR can fire and update both. The result: `raw` from cycle N and `value` from cycle N+1 are reported together. For telemetry this produces a momentarily inconsistent display. For calibration functions (which rely on `raw` and `value` readings), this could occasionally produce misleading output.

The boolean flags `see_left_wall`, `see_front_wall`, `see_right_wall` are single-byte `volatile bool` — these reads **are** individually atomic on AVR and correctly declared.

### [MEDIUM] `adc.get_dark()` / `adc.get_lit()` — no ATOMIC guard on 16-bit read

Already identified in the hardware abstraction review, confirmed here:

```cpp
// adc.h:162-164
int get_dark(const int i) const {
    return m_adc_dark[i];  // volatile int, 16-bit on AVR, no ATOMIC
}
```

`m_adc_dark` is `volatile int[8]`. Reading a 16-bit `volatile int` on AVR is not atomic — the ADC ISR can update the high byte between the two 8-bit load instructions. Callers: `Battery::update()` (`battery.h:39`), `Switches::update()` (`switches.h:52`), `Reporter::show_adc()` (`reporting.h:406`). For battery and switch channels (6 and 7), which change slowly, a torn read produces a one-tick error at most. Low observable impact but technically a data race. Compare with `get_raw()` which correctly uses `ATOMIC` (`adc.h:168-171`).

### [MEDIUM] `volatile float m_cross_track_error` / `m_steering_adjustment` — no ATOMIC on reads

```cpp
// sensors.h:323-324
volatile float m_cross_track_error = 0;
volatile float m_steering_adjustment = 0;
```

These are read from main context by `reporter.report_sensor_track()` → `sensors.get_cross_track_error()` and `sensors.get_steering_feedback()`:
```cpp
// sensors.h:116-120
float get_steering_feedback() { return m_steering_adjustment; }
float get_cross_track_error() { return m_cross_track_error; }
```

No `ATOMIC` block is used. On AVR, a `float` (32-bit) read is four 8-bit loads — the ISR can preempt between any of them. Reported values can be momentarily corrupt. Impact: reporting-only; these values are not used for control decisions in main context. Low operational risk, but inconsistent with the ATOMIC pattern used elsewhere in the codebase for `volatile float`.

### Correctly atomic: encoder delta counters

```cpp
// encoders.h:142-147
void update() {
    int left_delta = 0;
    int right_delta = 0;
    ATOMIC {
        left_delta = m_left_counter;
        right_delta = m_right_counter;
        m_left_counter = 0;
        m_right_counter = 0;
    }
```

`m_left_counter` and `m_right_counter` are `int` (16-bit), written by the encoder pin-change ISRs and read+cleared by `encoders.update()` inside the systick ISR. Despite systick being `ISR_NOBLOCK` (allowing encoder ISRs to preempt it), the `ATOMIC` block here correctly prevents torn reads of the 16-bit counters. **Correctly implemented.**

### [LOW] Unnecessarily volatile: `m_fwd_change` / `m_rot_change`

```cpp
// encoders.h:229-230
float m_fwd_change = 0;  // not volatile — correctly
float m_rot_change = 0;  // not volatile — correctly
```

These are written by `encoders.update()` (in systick ISR) and read by `motors.update_controllers()` via `robot_fwd_change()` and `robot_rot_change()` (also in systick ISR, same invocation, later in the same function). The encoder pin-change ISRs do not write to `m_fwd_change` or `m_rot_change`. There is no actual concurrency hazard. The `ATOMIC` blocks inside `robot_fwd_change()` and `robot_rot_change()` are therefore **unnecessary** — they temporarily disable interrupts for variables that have no concurrent writer at the point of access. This is harmless but imposes a small performance cost (6 machine instructions overhead per ATOMIC block), and it may inhibit ISR re-enabling during that window.

### [LOW] `static volatile bool` in encoder ISR callbacks — correct but over-qualified

```cpp
// encoders.h:88-89
void left_input_change() {
    static volatile bool oldA = false;
    static volatile bool oldB = false;
```

`oldA` and `oldB` are local statics, written and read only within `left_input_change()` (called from INT0 ISR). Since standard (non-NOBLOCK) ISRs are non-reentrant on AVR, these variables cannot be accessed concurrently. The `volatile` qualifier prevents the compiler from caching them in registers across iterations (which it could not do with statics anyway). Harmless, but unnecessary.

### [MEDIUM] `.noinit` Maze: `m_mask` uninitialised on cold power-on

```cpp
// config.h:142
#define PERSISTENT __attribute__((section(".noinit")))
// mazerunner-core.ino:41
Maze maze PERSISTENT;
```

Objects in `.noinit` are **never zero-initialised** by the AVR C runtime startup code, and **in-class initialisers do not run** for them. The `Maze` class has:
```cpp
MazeMask m_mask = MASK_OPEN;  // maze.h:524 — does NOT initialise in .noinit
```

On a cold power-on (SRAM content unpredictable), `m_mask` can be any value. `is_exit()` tests `(walls.X & m_mask) == EXIT`. With a random `m_mask`, wall states are interpreted incorrectly: what should be a WALL could appear as an EXIT or vice versa.

`m_walls[][]` is also random — each `WallInfo` byte can hold any 2-bit combination per wall. The `update_wall_state()` guard (`maze.h:327-353`) checks if the wall is `UNKNOWN` before writing:
```cpp
if ((m_walls[cell.x][cell.y].north & UNKNOWN) != UNKNOWN) {
    return;  // don't update a seen wall
}
```
`UNKNOWN = 0b10`. A random wall byte could have any value 0–3 (EXIT, WALL, UNKNOWN, VIRTUAL). If random data sets a wall to `EXIT (0)` or `WALL (1)`, `update_wall_state()` will never correct it — those walls are treated as already-seen and trusted. The robot could map phantom walls or miss real ones from the start of its first run.

`setup()` calls `maze.set_goal(GOAL)` after the button-check, so the goal location is always correctly set. However, `m_mask` is not reset in `setup()` — it is only reset inside `maze.initialise()` (called on button-held reset).

**[MEDIUM]** Reliable startup without button-held clear requires that SRAM content happens to be consistent from a previous power session. This is not guaranteed across battery changes. The README states the maze is stored in "EEPROM", which is incorrect — it is `.noinit` SRAM, which is volatile on power loss. Users who follow the documentation as written are misled about the reliability of maze persistence.

---

## Summary Table

| # | Severity | Issue | Location |
|---|---|---|---|
| 1 | **HIGH** | `args.argv[1]` and `args.argv[2]` accessed without checking `args.argc` — dereferencing uninitialized pointer is UB | `cli.h:349, 353, 403` |
| 3 | **MEDIUM** | No stack overflow detection on AVR; no worst-case stack depth analysis performed; 136-byte Queue on stack of a deeply nested call chain in a 2 KB system | `maze.h:430`, `queue.h:82` |
| 4 | **MEDIUM** | Queue overflow in `Maze::flood()` silently drops BFS frontier cells, producing incorrect cost maps; queue size of 64 is unverified against worst-case maze topology | `queue.h:55-57`, `maze.h:429` |
| 5 | **MEDIUM** | `volatile SensorChannel` struct fields read non-atomically from main context; ISR can update `raw` and `value` between the two reads | `sensors.h:95-98`, `reporting.h:205-209` |
| 6 | **MEDIUM** | `adc.get_dark()` / `adc.get_lit()` read 16-bit `volatile int` without ATOMIC — data race with ADC ISR; inconsistent with `get_raw()` which is correctly guarded | `adc.h:162-164` |
| 7 | **MEDIUM** | `volatile float m_cross_track_error` / `m_steering_adjustment` read in main context without ATOMIC; 4-byte float reads on AVR are non-atomic | `sensors.h:116-120, 323-324` |
| 8 | **MEDIUM** | `.noinit` Maze: `m_mask` and `m_walls[][]` are uninitialised on cold power-on (in-class initializers do not run for `.noinit` objects); random wall data prevents correct maze operation without button-held clear | `maze.h:524`, `config.h:142`, `mazerunner-core.ino:41` |
| 9 | **LOW** | `encoders.m_total_left` / `m_total_right` are `int` (16-bit, ±32767) with no overflow protection; noted as intentional for short runs only | `encoders.h:235-236` |
| 10 | **LOW** | `ATOMIC` blocks in `robot_fwd_change()` / `robot_rot_change()` are unnecessary — these variables have no concurrent writer at their point of use | `encoders.h:198-214` |
| 11 | **LOW** | `static volatile bool oldA/oldB` in encoder ISR callbacks — `volatile` qualifier is unnecessary for non-reentrant ISR-local statics | `encoders.h:88-89, 100-101` |
| 12 | **LOW** | `adc.h:237`: `uint8_t m_index = 0` is declared as a private member but never read or written — dead field | `adc.h:237` |
