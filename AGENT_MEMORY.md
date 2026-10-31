# Adeept RaspTank Pro — Durable Memory and Working Notes

Updated: 2026-10-31

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

IMU, switches, camera, radar scans, servo threads, web-server control and
persistent calibration are not implemented by the new façade. Never run
legacy scripts concurrently with new controllers that own the same hardware.

## Validation and remaining physical work

Mock-based `unittest` checks cover exact pin/PWM/direction behavior, ranges,
disabled operation, lazy imports, echo deadlines, ownership, partial
initialization and cleanup failure handling. Both existing notebooks are
valid JSON/Python and default to hardware-independent discovery/symbolic
checks. Hardware actions require disabled-by-default opt-in flags and
`finally` cleanup.

No physical Raspberry Pi, motor, servo, sensor, LED or OLED tests have been
performed. Mock/static success does not establish wiring, electrical safety,
physical direction, pulse calibration or timing accuracy. README documents
manual stand-supported tests and a physical emergency power cutoff.

## Durable conventions

1. Treat legacy hardware code as the wiring specification, not architectural
   speculation; distinguish deliberate safety corrections from equivalence.
2. Keep controllers synchronous, imports inert, optional devices explicit,
   and shared hardware ownership unambiguous.
3. Validate commands before output; clean up only acquired resources, attempt
   every resource on failure, and preserve the original initialization error.
4. Keep calibration transient unless persistence is separately requested.
5. Use existing standard-library unittest mocks and notebook JSON/AST checks;
   never imply physical validation from software-only results.
