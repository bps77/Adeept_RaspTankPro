# Adeept RaspTank Pro — Durable Memory and Working Notes

This file is a durable memory record for the assistant working on the Adeept RaspTank Pro project.
It is meant to capture project goals, design decisions, architecture findings, and working notes in a human-readable format.

## Project Goal
The long-term goal is to transform the repository from a script-heavy, client/server robot control project into a cleaner, more Pythonic robotics library centered on a single `RaspTank` object.

The refactor should preserve the existing hardware functionality and the real Raspberry Pi wiring while making the robot easier to learn from, test, and extend in VS Code, Jupyter notebooks, and interactive Python sessions.

Key priorities:
- preserve actual hardware behavior
- keep the public API simple and intuitive
- separate hardware drivers from user-facing robot logic
- support interactive experimentation
- provide strong safety/resource management
- keep legacy behavior available where useful

## Repository Context
Repository: `adeept/Adeept_RaspTankPro`
User fork: `bps77/Adeept_RaspTankPro`
Current working branch: `refactor-rasptank-core`

## What the Repository Actually Does
The existing repository is a robot control project built around a Raspberry Pi and a robot controller stack. It is not yet a modern Python package.

The repo includes:
- a server-side control loop in `server/server.py`
- GPIO motor logic in `server/move.py`
- servo control in `server/servo.py` and `server/RPIservo.py`
- ultrasonic detection in `server/ultra.py`
- WS2812 LED driver in `server/LED.py`
- OLED display code in `server/OLED.py`
- GPIO switch outputs in `server/switch.py`
- OpenCV/FPV vision modules
- a GUI client in `client/GUI.py`
- auto-setup scripts in `setup.py`

The central design pattern is:
- the Raspberry Pi runs the robot logic
- the client sends commands over a TCP socket
- the server receives commands and calls GPIO/PCA9685 functions
- the robot runs actions such as forward/backward motion, servo control, camera/FPV use, and automatic modes

## Important Hardware Findings
These are based on actual code, not guesses.

### Motors
File: `server/move.py`

Motor pins:
- `Motor_A_EN = 4`
- `Motor_B_EN = 17`
- `Motor_A_Pin1 = 26`
- `Motor_A_Pin2 = 21`
- `Motor_B_Pin1 = 27`
- `Motor_B_Pin2 = 18`

Movement logic:
- `move(speed, direction, turn, radius)`
- direction values: `'forward'`, `'backward'`, `'no'`
- turn values: `'left'`, `'right'`, `'no'`
- `motorStop()` disables all direction pins and EN pins

This is the core low-level movement system we must preserve.

### Servos
Files: `server/servo.py`, `server/RPIservo.py`

The robot uses a PCA9685 PWM board.
- PCA9685 configured at 50Hz
- values are treated as PWM pulse values, not angle degrees directly
- servo directions are sign-flipped by flags like `pwm0_direction`, `pwm1_direction`, etc.

Observed initialization:
- `pwm0_init = 300`
- `pwm0_max = 450`
- `pwm0_min = 150`
- other channels differ by servo output

The behavior is highly hardware-specific and should remain intact.

### Ultrasonic Sensor
File: `server/ultra.py`

Pins:
- Trigger: BCM 11
- Echo: BCM 8

`checkdist()` sends a trigger pulse, waits for echo, and calculates distance based on elapsed time.

### LED Strip
File: `server/LED.py`

- GPIO 12
- 16 pixels
- `rpi_ws281x` library
- color wipe functions set pixels and show them

### OLED Display
File: `server/OLED.py`

- I2C port 1
- address `0x3C`
- uses `luma.oled.device.ssd1306`

### GPIO Switch Outputs
File: `server/switch.py`

- BCM 5, 6, 13
- used as general-purpose switched outputs

### IMU
The code checks for an MPU6050 at I2C `0x68` and uses it for motion-related features in `server.py` and `functions.py`.

### Camera and FPV
Files includes:
- `server/FPV.py`
- `server/camera_opencv.py`
- `server/base_camera.py`

These modules implement vision and the FPV stream pipeline.

## Architecture and Design Lessons
### What is good in the current repo
- hardware-specific logic is concrete and grounded in actual GPIO use
- each module handles one category of hardware
- movement and servo functions are conceptually separated
- there is a clear robot behavior model even though it is script-based

### What is fragile
- modules are not object-oriented or easy to import interactively
- the repo mixes low-level drivers, network control, and app behavior in one place
- server code is tightly coupled to socket messages and long-running control loops
- there is no single `RaspTank` object that matches how a developer naturally thinks
- there are multiple servo implementations with overlapping responsibilities

## Refactor Principles
The following principles are now the working standard for this project:

1. Preserve working hardware behavior over code elegance.
2. Inspect real source before changing anything.
3. Prefer explicit, understandable Python APIs over hidden magic.
4. Separate hardware implementation from robot control logic.
5. Keep a compatibility layer for legacy scripts when appropriate.
6. Make resource cleanup and safety mandatory.
7. Make the package suitable for VS Code and notebook-based interactive use.

## Working Plan
### Phase 1: inventory and hardware mapping
Completed.

### Phase 2: low-level motor abstraction
Next planned step.
We will wrap `server/move.py` into a `MotorController` or `Motors` object that preserves:
- BCM pin assignments
- movement directions
- stop behavior
- speed limits
- cleanup logic

### Phase 3: servo abstraction
Wrap the servo behavior from `server/servo.py` into a `ServoController` with:
- safe ranges
- move methods
- initialization helpers
- explicit cleanup

### Phase 4: sensor wrappers
Add wrappers for ultrasonic, LED, display, and IMU.

### Phase 5: central `RaspTank` class
Create a single object that exposes:
- `forward()`, `backward()`, `stop()`, `turn_left()`, `turn_right()`
- `motors`, `servos`, `camera`, `ultrasonic`, `leds`, `display`, `imu`
- `close()` and context manager support

### Phase 6: docs, notebooks, and tests
Provide strong educational material and keep examples practical.

## Important Design Decisions
### Decision: do not rewrite everything in one pass
This would risk breaking hardware behavior and make debugging harder.

### Decision: keep the physical GPIO mapping as the source of truth
The actual source code is the real specification.

### Decision: create educational docs before the package refactor is complete
This helps new developers understand the repo while changes are happening.

### Decision: maintain a compatibility layer, not a total replacement
Some old control patterns may still be useful as optional wrappers.

## Open Questions / Unresolved Items
- Which camera driver is the primary one used on the real robot?
- Are all servo outputs always present, or only some are installed depending on hardware revision?
- Which functions are meant to be optional vs required?
- Which legacy network/system behaviors should remain in compatibility mode?

## Notes for Future Work
At the end of the refactor, the project should feel like:

```python
from rasptank import RaspTank

tank = RaspTank()
tank.forward(30)
tank.stop()
print(tank.ultrasonic.distance())
```

This should be a readable, safe, and friendly API for robotics learning.

## Conclusion
The repository is a functional hardware project with real Raspberry Pi GPIO wiring, not just an abstract concept. The correct approach is to preserve that behavior while wrapping it in a better Python interface.

The documentation and refactor should be built around transparency, learning, and safe experimentation.

---
Project memory version: 1
Updated: 2026-10-30

