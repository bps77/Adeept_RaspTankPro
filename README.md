# Adeept_RaspTankPro

Example programs for Adeept RaspTank Pro.

## Repository independence

The robot server does not automatically fetch, reset, or pull repository code,
or reboot to apply updates. Updates must be performed manually from your own
repository. The legacy `production` setting no longer controls automatic updates.
Wi-Fi detection and access-point fallback remain unchanged.

Removing a Git upstream remote or disabling automatic updates does not detach
GitHub's fork relationship. The repository owner must detach the fork through
GitHub's available fork-detachment options or contact GitHub Support. Preserve
the existing history and attribution when doing so.

## Synchronous Python API

Run from the repository root (Python 3). Importing `rasptank` or any of its
hardware modules does not import Pi drivers or open devices. No new dependencies
are needed for dry-run checks; real hardware uses the existing Pi drivers.
The legacy `server/` scripts are unchanged and are not imported by this API.
Do not run them concurrently with these controllers.

```python
from rasptank import RaspTank

with RaspTank(use_gpio=False, enable_servos=True) as tank:
    tank.forward(15)          # validates the command; no hardware output
    tank.turn_left(15)
    tank.stop()
    tank.servos.set_pulse(0, 300)
    tank.servos.move_relative(0, 10)  # symbolic position is now 310
```

`RaspTank(use_gpio=True)` explicitly initializes **only stopped motors**.
`enable_servos`, `enable_ultrasonic`, `enable_leds` and `enable_display` default
to `False`; their attributes are `None` unless enabled or injected. Enabling
servos sets frequency but never centers them. `use_gpio=False` propagates to
every created controller (including I2C/LED devices). Injected controllers,
such as `RaspTank(motors=my_controller)`, are borrowed, retain their own hardware
mode, and must be closed by their caller. Created controllers are owned; failed
construction rolls them back and `close()` attempts every owned resource.
Context managers clean up even when the body fails.

### Wiring and commands

* Motors use BCM A/right enable **4**, direction **26/21**; B/left enable
  **17**, direction **27/18**, with PWM at 1000 Hz. Forward sets both first
  direction pins high and second pins low; backward reverses them. Raw left
  forward is direction `1`, raw right forward is `0`, as in `server/move.py`.
* `forward`, `backward`, `turn_left`, `turn_right` default to speed **60**
  (the legacy manual example). `move(speed, direction, turn, radius=0.6)` accepts
  directions `forward/backward/no` and turns `left/right/no`. Speed is finite
  0–100 duty percent and radius is finite **0 < radius <= 1**. Validation occurs
  before outputs. Legacy turning while driving **stops the inner track**;
  radius does not make a curved-drive algorithm. Turn-only commands spin.
* `MotorController(use_gpio=False)` is a dry-run. Its explicit `setup()` may
  enable hardware later, preserving existing test/API compatibility. Setup
  starts each PWM once at **zero duty**, never at the legacy unsafe 100%.
  Stopping sets duty and driver outputs low. Close stops PWM and cleans up only
  successfully acquired pins, never global `GPIO.cleanup()`.
* Servos use PCA9685 **0x40**, bus **1**, **50 Hz** (from `RPIservo.py`).
  Channels **0..4** use `servo.py` ranges **150–450, 160–480, 100–500,
  100–500, 100–500** and direction flags **1, 0, 1, 1, 0**.
  `set_pulse(channel, pulse)` uses integer PCA9685 ticks, not degrees.
  `move_relative(channel, steps)` adds signed ticks for direction 1 and
  subtracts for direction 0. Out-of-range commands raise `ValueError`, not
  silent clipping. Initial positions are **assumed 300**, not read-back.
  Establish a known physical position with an explicit pulse before relative
  motion. `calibrate(channel, pulse)` changes only an instance's initial value;
  `center(channel)` explicitly commands it. No constructor servo motion occurs.
  Close disables only channels commanded by that controller using PCA9685
  full-off, never `set_all_pwm`. Avoid multiple owners of one channel or board.
* Ultrasonic trigger/echo are BCM **11/8**. `distance()` returns **metres**
  rounded to two decimals with the legacy 340 m/s sound speed. Both echo
  waits have independent monotonic deadlines; default timeout **0.03 s** is
  a bounded API policy, not legacy calibration. `SensorTimeoutError` indicates
  a missing/late edge. Dry-run readings raise `HardwareDisabledError`.
* `LEDStrip` defaults to the legacy class's **16 pixels**, BCM **12**,
  800 kHz, DMA 10, brightness 255, channel 0. `color_wipe(r, g, b)` validates
  integer RGB 0–255, sets all pixels and shows **once**, not per-pixel animation.
  Close turns it off and uses native release where the driver exposes it.
* `OledDisplay` uses bus **1**, address **0x3C** and `show(line1, line2)` draws
  two lines synchronously. It is not a six-line/threaded legacy replacement;
  `close()` invokes native device cleanup (or serial cleanup on failed creation).

All controllers have idempotent `close()` and reject operations after close.
Cleanup is best-effort across resources and reports the first failure.
Physical power disconnection remains the emergency stop if software or the
process fails. Low-level controllers accept `gpio=`, servo `pwm=`, LED
`strip=/color=`, or OLED `device=/canvas=` for mocked tests; disabled mode never
touches those injected driver objects.

### Calibration and limits

New controller positions, RGB/text state and servo calibration are **transient
instance state**, not persistent storage. This API neither loads nor rewrites
legacy calibration files or source-based calibration (including `RPIservo.py`).
It does not implement legacy servo threads, radar scans, IMU, switches, camera,
or web-server features. No physical Raspberry Pi validation has been performed.

### Manual Pi validation (explicit opt-in)

1. Stop the legacy server and all other device owners. Select the Pi's Python
   kernel, confirm existing drivers are installed, and enable I2C through your
   Pi's normal configuration if testing I2C devices. Imports are safe checks,
   but driver availability alone does not prove correct wiring.
2. Check the BCM mapping, common ground, motor/servo power, and sensor voltage
   compatibility. Secure the chassis on a stand with tracks clear; do not rely
   on a table to prevent a fall. Keep a physical power cutoff within reach.
3. First initialize stopped motors, then intentionally run one tiny low-duty
   movement. Low duty may stall rather than move. Watch direction and stop at
   once if anything is unexpected. Always use `finally` cleanup:

   ```python
   from rasptank import RaspTank
   import time

   RUN_HARDWARE_TEST = False  # change only after the checks above
   if RUN_HARDWARE_TEST:
       tank = None
       try:
           tank = RaspTank(use_gpio=True)
           tank.forward(15)
           time.sleep(0.3)
           tank.stop()
       finally:
           if tank is not None:
               tank.close()
   ```

4. Test optional devices separately, explicitly enabling only the device needed.
   For servos, verify linkage clearance and a known safe absolute pulse before
   relative commands; no automatic centering is provided. For ultrasonic,
   compare metre readings to a known distance and test missing echo timeouts.
5. Both existing notebooks default to hardware-independent discovery and
   symbolic checks. Their real-motion cells are disabled; read their safety
   gates before changing flags. Software mocks do not verify physical behavior.

Run software validation without any Pi:

```sh
python -m unittest discover -s tests
```