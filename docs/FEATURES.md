# Legacy feature parity and roadmap

Status of every feature of the original Adeept server (`server/`) versus the
synchronous `rasptank` package. **None of the ported code has run on physical
hardware**; all checks are mock-based (`python -m unittest discover -s tests`).

## Ported (synchronous, import-inert, dry-run capable)

| Legacy source | New API | Notes / deliberate differences |
|---|---|---|
| `move.py` | `MotorController`, `RaspTank.move/forward/...` | PWM starts at 0 duty, not 100% |
| `servo.py`, `RPIservo.py` (basic) | `ServoController` | Explicit pulses; no threads, no calibration file |
| `ultra.py` | `UltrasonicSensor` | Bounded timeouts; metres, 340 m/s |
| `LED.py` | `LEDStrip.color_wipe` | One frame, not per-pixel animation |
| `robotLight.py` setSomeColor/police/breath | `LEDStrip.set_pixels/police/breath/play` | Bounded cycles, caller owns timing; police lights pixels 0..12 |
| `OLED.py` | `OledDisplay` | Two lines, no background thread |
| `switch.py`, `robotLight.py` switches | `SwitchPorts` (BCM 5/6/13, head light = port 1) | `front_light(False)` clears ports 2+3; legacy cleared 1+3 (typo) |
| `findline.py` | `behaviors.simple_line_step`, `LineSensors` | One step; caller loops |
| `functions.py` trackLine | `behaviors.line_follow_step` | Same state table, brake kick and sleeps |
| `functions.py` Automatic | `behaviors.obstacle_step` | Reads distance once; stops tank on sensor failure |
| `functions.py` radarScan | `behaviors.radar_scan` | Uses controller ranges (160..480 on channel 1); timeouts skipped |
| `functions.py` Steady, `mpu6050Test.py` | `behaviors.SteadyStabilizer`, `Mpu6050` | Output clamped to servo range (legacy did not) |
| `Kalman_filter.py`, `PID.py` | `KalmanFilter`, `PID` | PID clock injectable |
| `info.py` | `rasptank.system` | Fixes legacy missing `import os`; psutil imported lazily |

## New optional camera and supervised feature layer

Imports remain inert. Unlike synchronous core controllers, these workers are
explicitly started and supervised. See [usage/setup](CAMERA_USAGE.md).

| Reference / feature | New API | Scope and limits |
|---|---|---|
| `camera_opencv.py` Picamera2 capture | `CameraController`, `RaspTank.start_camera` | Isolated process, fresh immutable RGB/JPEG, bounded startup/shutdown; supports older Bullseye capture API with exposure-time checks; no legacy imports |
| Notebook video | `notebook.show_camera`, `RaspTank.show_camera` | Nonblocking optional widgets, one latest frame; opt-in `popout=True` browser window shares JPEG payload (anywidget); camera remains borrowed |
| `camera_opencv.py` HSV color search | `vision.detect_color`, `hsv_target`, `annotate` | Detection only; no motor/servo/light side effects |
| Semantic class identification | `vision.MobileNetSSD`, `detect_objects`, `find_object` | Verified explicit local VOC model; bounded search process; custom detector contract |
| Fused room roaming | `FloorObserver`, `RoamPolicy`, `RaspTank.start_roaming` | Calibrated corridor and range both veto forward; finite pulses/turn recovery/duration, no reverse/SLAM |
| Driving watchdog and arbitration | `RaspTank(command_timeout=...)`, `stop`, `reset_drive_fault` | Opt-in command expiry, manual override and latched faults; low-level calls bypass it |

Floor-color assessment is not guaranteed obstacle/free-space recognition.
Pi/physical acceptance and frontend validation remain outstanding; no numerical
latency or stopping-distance default is claimed safe on hardware.

## Not ported (documented only)

| Legacy feature | Source | Why / what is needed |
|---|---|---|
| Legacy camera-driven servo tracking, CV line-follow and motion/watchdog modes | `FPV.py`, `FPVtest.py`, `camera_opencv.py` | New independent capture/color helpers do not reproduce these hardware side effects |
| Flask web UI and websocket control | `webServer.py`, `app.py`, `appserver.py`, `server.py`, `dist/` | Network-exposed control needs authentication design |
| Tkinter desktop client | `client/GUI.py` | Remains legacy |
| Threaded servo motion modes (init/auto/certain/quick/wiggle, speed ramps) | `RPIservo.py` | Would need a supervised background worker with stop-on-close |
| Persistent servo calibration | `RPIservo.py` rewrites `init_pwmN` | Separately requested persistence only |
| Wi-Fi hotspot fallback, autorun | `wifi_hotspot_manager.sh`, `autorun.py` | System-level; unchanged |
| Look-up/down helpers, `initPosServos.py` | `servo.py` | Use `ServoController.move_relative/center` |

## Possible future features

The [camera and autonomy implementation plan](CAMERA_AUTONOMY_PLAN.md)
records delivery boundaries and remaining physical acceptance gates.

* Curved-drive (radius-aware) motion; speed ramps and acceleration limits.
* Persistent, opt-in calibration file with schema and validation.
* General-purpose supervised runner for the other synchronous `behaviors` steps.
* Battery voltage monitoring, if the board exposes an ADC.
* Line-follower PID steering using `PID`; IMU heading hold using the gyro.
* Obstacle mapping from `radar_scan` output; mock-hardware simulator.
* Authenticated, read-only telemetry endpoint built on `rasptank.system`.

## Safety and testing notes

Run stand-supported (wheels off the ground) with an emergency power cutoff.
Verify first: switch pins, line-sensor polarity (1 = line), MPU6050 address and
axis, radar angle sign, servo channel mapping (radar = 1, steady = 4), and that
behaviors stop the motors on failure. Never run legacy `server/` scripts at the
same time as these controllers.
