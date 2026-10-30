# Adeept RaspTank Pro — Durable Memory and Working Notes

This file is a durable memory record for the assistant working on the Adeept RaspTank Pro project.
It captures project goals, design decisions, architecture findings, and working notes.

## Project Goal
Transform the repository from a script-heavy, client/server robot project into a cleaner, more Pythonic robotics library centered around a single `RaspTank` object while preserving the actual Raspberry Pi hardware behavior.

## Repository Context
- Repository: `adeept/Adeept_RaspTankPro`
- Fork: `bps77/Adeept_RaspTankPro`
- Working branch: `refactor-rasptank-core`

## Actual Hardware Behavior Understood So Far
### Motors
File: `server/move.py`
- BCM pins: `4`, `17`, `26`, `21`, `27`, `18`
- Movement logic is centralized in `move(speed, direction, turn, radius)`
- `motorStop()` is the correct shutdown behavior during emergency or cleanup events

### Servos
Files: `server/servo.py`, `server/RPIservo.py`
- PCA9685 at 50Hz is used for servo motion
- servo values are not arbitrary floating values; they are hardware pulse values and direction-sensitive

### Ultrasonic Sensor
File: `server/ultra.py`
- Trigger: BCM 11
- Echo: BCM 8
- Distance is calculated by pulse timing

### LED, OLED, Switch, IMU
- LED strip uses GPIO 12 and `rpi_ws281x`
- OLED uses I2C at `0x3C`
- switches use BCM 5, 6, 13
- IMU is an MPU6050 at `0x68`

## Refactor Principles
1. Preserve working behavior over architectural elegance.
2. Treat the existing code as the hardware specification.
3. Prefer explicit Python APIs over hidden background behavior.
4. Keep hardware details understandable for learners.
5. Keep compatibility paths when legacy code remains useful.
6. Make cleanup and safety explicit.

## Progress Notes
### Current state
- Added the `rasptank` package root and config constants.
- Added hardware exception types.
- Added a first low-level `MotorController` wrapper preserving the `server/move.py` logic.
- Added a top-level `RaspTank` façade for interactive use.
- Added a `ServoController` wrapper around the original PCA9685 behavior.
- Added an `UltrasonicSensor` wrapper around the original timing-based distance logic.
- Added mock-based unit tests for the initial motor and servo abstraction layers.

### Review and critique
Static review performed on the code created so far:
- Importing the package on a non-Raspberry-Pi environment will fail because `RPi.GPIO` and `Adafruit_PCA9685` are imported at module load time.
- The cleanup code needs to be explicit and defensive to avoid overly aggressive GPIO cleanup.
- The public facade should remain intentionally simple; broad abstraction would hide the real hardware behavior.
- Documentation needs to explain the actual hardware mapping and refactor choices for learners.

### Corrective actions taken
- Preserved original GPIO and PWM logic instead of replacing it with speculation.
- Kept the wrappers thin and explicit rather than over-engineering them.
- Emphasized beginner-friendly explanations in the HTML guide and memory notes.
- Used mock-based testing to validate early refactor steps without real hardware.

## Action Plan
1. Keep the motor abstraction as the safe baseline.
2. Keep the servo abstraction tied to the original PCA9685 ranges and behavior.
3. Keep the ultrasonic sensor behavior true to the original pulse timing logic.
4. Add the remaining optional wrappers for LEDs, OLED, IMU, and camera gradually.
5. Build a clearer user-facing documentation layer around the object model.
6. Validate with mocked/unit-test style checks and static review.

## Important Policy for This Work
- Do not break actual robot behavior.
- Do not rewrite the repo in one pass.
- Document the reasoning and the decisions.
- Keep the human-readable explanation accessible to young learners.

---
Project memory version: 3
Updated: 2026-10-30
