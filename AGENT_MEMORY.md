# Adeept RaspTank Pro — Durable Memory and Working Notes

Updated: 2026-11-01

## Implemented code

The `rasptank` package implements synchronous motors, servos, ultrasonic, LED
and two-line OLED wrappers plus a `RaspTank` façade. `config.py` contains
explicit legacy wiring/ranges and `exceptions.py` defines lifecycle/hardware
failures. It is not a replacement for every legacy server feature.

* Imports do not load hardware drivers or open devices; drivers load only when
  enabled controllers are constructed or motor/sensor `setup()` is explicit.
* `use_gpio=False` disables all hardware access, including I2C and LEDs.
  Dry-run sensor reads raise instead of inventing data.
* Motor mapping and movement direction/turn decisions follow `server/move.py`.
  Both PWM outputs start once at zero duty. Driving turns stop the inner track;
  radius is validated but does not implement curved-drive behavior.
* Servo channels 0..4 retain `server/servo.py` ranges/direction flags. The
  PCA9685 address/bus/frequency follow `server/RPIservo.py`: 0x40/1/50 Hz.
  No servo channel is commanded at construction; pulse/relative/center
  operations are explicit. Close releases only commanded channels.
* Ultrasonic uses BCM 11/8, monotonic deadlines on both echo waits, and metres
  rounded to two decimals with the legacy 340 m/s constant.
* LEDs show one synchronous frame; OLED shows two lines, not the legacy
  six-line background thread. Optional façade devices are opt-in.
* Cleanup is scoped, idempotent and best-effort. GPIO cleanup never uses the
  global form; acquired PWM instances are stopped. Closed controllers reject
  commands. The façade rolls back partial creation and closes every owned
  resource; injected controllers are borrowed and remain caller-owned.

## Ported feature layer (software-only validated)

`SwitchPorts` (BCM 5/6/13), `LineSensors` (L20/M16/R19, 1 = line), `Mpu6050`
(0x68), LED `police/breath/set_pixels`, `KalmanFilter`, `PID`, `rasptank.system`
and `rasptank.behaviors` (line follow, obstacle step, radar scan, steady
stabiliser) mirror `server/` logic as single bounded steps with injectable
sleeps; callers own loops. Behaviors stop motors on failure. Deliberate fixes:
front-light off clears ports 2+3, steady output is clamped, info.py `os` import.
Façade opt-ins: `enable_switches/line_sensors/imu`; injected remain borrowed.

## Runtime state versus persistence

Servo positions begin as assumed 300-tick values, not physical measurements.
Positions, transient initial calibration, LED color and OLED text belong to
each instance. `calibrate()` changes an in-memory initial pulse only and
`center()` is an explicit movement, not constructor behavior.

The new API does not load, rewrite or create calibration files. Legacy
calibration/source persistence remains in the unchanged `server/` path;
it is separate from these transient controllers. Do not infer physical servo
position from symbolic state or expect calibration to survive a restart.

## Legacy compatibility and scope

Legacy server scripts remain unchanged by the core refactor. Prior removal
of automatic fetch/reset/pull/reboot updates remains in place; Wi-Fi/AP
fallback is unchanged. Fork detachment is an owner/GitHub operation.

Web-server control, legacy camera-driven servo/motion modes, servo threads/modes
and persistent calibration are not implemented by the new package; see
`docs/FEATURES.md`. Never run
legacy scripts concurrently with new controllers that own the same hardware.

## Optional camera, vision and supervised driving

`CameraController` is opt-in and does not capture at construction. Picamera2
lives in a spawned process; `capture_request()` plus explicit release supports
older Bullseye Picamera2. SensorTimestamp minus exposure duration rejects
pre-read exposures; missing timing fails explicitly. Pillow conversion
defines packed RGB, and one immutable latest `Frame` supplies consumers.
Frame deadlines, source errors and cleanup errors are explicit. Graceful
shutdown drains IPC and closes the source; hung drivers are terminated/killed.
Disabled mode never opens/invokes a source. Injected cameras remain borrowed.

`notebook.show_camera`/`RaspTank.show_camera` return optional rate-limited widget
handles; attached views close with the tank.
`popout=True` adds a lazy anywidget browser button; inline remains the default.
The pop-out shares one JPEG widget payload without another camera/server,
mirrors stop/error status and closes on connected-frontend view cleanup.
Closing its browser window does not stop capture or motors.
`vision` has HSV color matching, annotations, a checksum-verified local
Caffe MobileNet-SSD/VOC adapter and a
process-bounded `find_object`; no automatic model downloads or motor outputs.
OpenCV 4.x is required for Caffe; optional requirements and manual assets are
documented in `docs/CAMERA_USAGE.md`.

`command_timeout` explicitly enables facade arbitration/watchdog; None preserves
old synchronous motion. Manual facade output cancels autonomy. Stop invalidates
queued tokens, faults latch, close stops even borrowed motors without closing
them. Roaming requires both fresh sensors and an explicitly calibrated observer.
`FloorObserver` is a restrictive HSV corridor heuristic, not depth or guaranteed
obstacle avoidance. Each short pulse stops before post-stop observations;
recovery/duration are finite, no reverse/servo scan occurs. Watchdog is separate
from perception; a hung observer prevents restart and is reported on bounded
shutdown. GPIO/process/OS failures still require a physical cutoff.

## Validation and remaining physical work

Software `unittest` checks (139, incl. process/image/widget and opt-in model fixtures) cover exact pin/PWM/direction behavior, ranges,
disabled operation, lazy imports, echo deadlines, ownership, partial
initialization and cleanup failure handling. Both existing notebooks are
valid JSON/Python and default to hardware-independent discovery/symbolic
checks. Hardware actions require disabled-by-default opt-in flags and
`finally` cleanup.

No physical Raspberry Pi camera/roaming tests have been performed by this agent.
An actual pinned local MobileNet-SSD inference was executed on synthetic images,
not on the Pi. Core hardware checks remain software-only in this work; owner
feedback that existing notebooks ran successfully does not validate new camera,
vision or watchdog behavior. Mock/static success does not establish wiring, electrical safety,
physical direction, pulse calibration or timing accuracy. README documents
manual stand-supported tests and a physical emergency power cutoff.

## Durable conventions

1. Treat legacy hardware code as the wiring specification, not architectural
   speculation; distinguish deliberate safety corrections from equivalence.
2. Keep core controllers synchronous, imports inert, optional devices explicit,
   and shared hardware ownership unambiguous. Camera/view/autonomy workers start
   explicitly, have bounded shutdown and surface faults.
3. Validate commands before output; clean up only acquired resources, attempt
   every resource on failure, and preserve the original initialization error.
4. Keep calibration transient unless persistence is separately requested.
5. Use existing standard-library unittest mocks and notebook JSON/AST checks;
   never imply physical validation from software-only results.
6. Every public and private class, function and method keeps a docstring
   stating contract, units and failure behaviour; update it with any change.
7. Update README.md, docs/FEATURES.md and this file together when scope shifts.

## Documentation map

`docs/CAMERA_AUTONOMY_PLAN.md` records approved camera/notebook/vision and
supervised fused-roaming scope; `docs/CAMERA_USAGE.md` is the implemented API,
model/dependency setup and remaining physical acceptance guide.

README.md (API, wiring, manual validation, package layout), docs/FEATURES.md
(legacy parity/roadmap), this file (durable state), module/class/method
docstrings (contracts). Legacy `server/` and `client/` code is undocumented
beyond its own comments.
