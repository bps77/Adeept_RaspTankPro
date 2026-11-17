# Interactive wand tracking POC

Proves the existing Pi + Arducam IMX519 can track a bright wand tip and draw its trajectory. No spell recognition or home automation yet.

## Pieces

- `rasptank/wand.py`: `WandTracker` (frame in, `(x, y)` out) plus trajectory helpers `path_length`, `bounding_box`, `normalize`, `direction_changes`. It only needs RGB numpy frames, so it has no dependency on the tank or this camera.
- `camera_source(camera)` adapts the existing `CameraController` to that frame interface. `CameraController` itself is unchanged.
- `rasptank/wand_arm.py`: optional `WandArmFollower` that steps the arm's `pan` and `camera` joints toward the tracked wand through the existing `RobotArm`.
- `notebooks/interactive_wand_poc.ipynb`: setup, camera init, test frame and FPS, preprocessing view, live tracking, optional arm demo, cleanup.

Pipeline: RGB frame, grayscale, blur, threshold, optional motion filter, connected components, area filter, pick candidate (nearest to the last position within `MAX_JUMP`, else largest), append to trajectory.

## Run

Uses the dependencies in `requirements-camera.txt` (no new ones). On the Pi, in a kernel with Picamera2, stop any other camera user (e.g. the legacy web server), open the notebook, set `RUN_CAMERA = True` (the default opens no hardware) and run cells in order. The notebook uses dry-run motors, so no motor GPIO is touched. Hold a bright tip (phone flashlight, white LED, reflective tape) in front of the camera in a dim room. Stop with the button or by running the last cell, which always closes the camera and can be run at any time.

Expected: the camera feed shows a blue circle on the tip, a green trail, `Tracking: YES/NO`, FPS and point count.

## Diagnostics

- `rasptank/wand_diagnostics.py`: `SyntheticWand` (figure-8 tip plus a lamp, noise and dropouts), `benchmark_camera` (FPS, dropped frames, timeouts, latency, processing time, detection rate), `sweep_modes` (tries several size/FPS modes, closing the camera each time), `scene_report` (brightest pixel, percent over threshold, candidates, tuning hint) and `LiveView`.
- Notebook: `DEMO_SYNTHETIC = True` runs everything without a camera. Section 5 benchmarks, section 6 (`RUN_MODE_SWEEP`) compares modes, and section 8 shows a white trace on black with a `TRACE_DELAY` slowdown and a replay of the last finished stroke.
- Reading the numbers: capture FPS well below the request means the camera pipeline is the limit; process ms above frame time means tracking is; low detection rate means fix lighting or `THRESHOLD`.
- Strokes: `on_stroke` fires when the tip is lost after at least `min_stroke_points`; `resample()` gives fixed-length trajectories for later recognition.

## Arm demo (optional, moves servos)

Needs `ENABLE_ARM_FOLLOW`, `STAND_AND_POWER_CUTOFF_READY` and `SERVO_CLEARANCE_VERIFIED` set to `True`. The follower starts disarmed; a toggle arms it. It moves only while the wand is tracked, in small rate-limited steps (default 5 Hz, at most 5 ticks), ignores a 10% center dead band, counts joint limits instead of forcing past them, and disarms itself on any hardware error. The joints and directions come from the legacy source: pan (channel 0, arm left/right) carries the camera, and channel 4 is camera tilt. `server/FPV.py` moves `lookleft` when the target is left of center and `up` when it is above, and `server/servo.py` directions match `config.SERVO_DIRECTIONS`, so no inversion should be needed. Still test on a stand with a cutoff in reach, and flip `INVERT_X`/`INVERT_Y` if it turns away from the wand. Its logic is unit-tested with a mock PWM driver, not on real servos.

## Tuning (top of the notebook)

| Parameter | Effect |
|---|---|
| `THRESHOLD` | Brightness cutoff (0-255). Lower if the tip is missed; raise if background lights are picked up. |
| `MIN_AREA` / `MAX_AREA` | Blob size limits in pixels. |
| `MAX_TRAJECTORY_POINTS` | Trail length. |
| `MAX_JUMP` | Largest per-frame move accepted; stops hopping to other bright objects. |
| `MAX_MISSED_FRAMES` | Frames of dropout tolerated before tracking is lost. |
| `MOTION_THRESHOLD` | Set (e.g. 25) to ignore static bright objects. |
| `CAMERA_SIZE`, `CAMERA_FPS` | 640x480 at 30 by default (controller allows up to 60 FPS). Use the measured capture rate in section 3, not the requested one. |

## Limitations

- Visible-light only: lamps, windows and reflections look like wands. Works best in a dim room with a bright tip.
- Frames cross a process boundary as raw RGB plus a JPEG each (existing camera design), which may cap FPS below 60. Higher modes are untested on the IMX519 and depend on what libcamera selects.
- A mid-size lamp (below `MAX_AREA`) can be locked onto while acquiring.
- Autofocus and auto-exposure can shift brightness mid-stroke.
- One target only. Display runs at 10 FPS while tracking runs at camera speed.
- `opencv-python-headless` is pinned `<5` in `requirements-camera.txt`; the full test suite passes on OpenCV 4.14.

## Moving to a NoIR/IR camera

Add an IR illuminator and a reflective wand tip, supply frames through any `read() -> RGB ndarray` callable (USB camera, video file, another Picamera2 setup), and raise `THRESHOLD`. `WandTracker` and the trajectory code stay unchanged. Optionally add an IR-pass filter.

## Next steps

1. Measure FPS and tracking reliability on the Pi; record a video for repeatable tests.
2. Save labelled trajectories and use `normalize()` for template matching (for example a $1 recognizer) before any ML.
3. Map recognized gestures to events (Lumos, Nox, Accio) and then to home automation.
