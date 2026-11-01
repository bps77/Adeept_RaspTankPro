# Camera and supervised notebook driving

The camera, optional notebook view, local vision helpers and finite fused roaming
are implemented. They have software-only validation, including an actual local
MobileNet-SSD inference; no physical Pi camera, latency, stopping-distance or
floor-navigation validation has been performed.

## Environment

Run the notebook kernel **on the Pi**, locally, via Remote SSH, or through an
SSH-tunneled Jupyter session. A laptop kernel needs a separate remote control
protocol and is not supported. Do not expose Jupyter without its normal
authentication or open an internet-facing tank-control port. Stop legacy server
scripts before acquiring any device.

On a supported Raspberry Pi OS/libcamera installation, install the OS camera
stack with your normal package manager:

```sh
sudo apt install python3-picamera2 python3-opencv python3-numpy python3-ipywidgets
```

If using a virtual environment, create it with `--system-site-packages` so it
can see Picamera2/libcamera. Select that environment as the notebook kernel.
`requirements-camera.txt` is the pip alternative for numpy/OpenCV/widgets;
it does not install Pi camera drivers. Prefer the OS OpenCV package on the Pi
if a compatible pip wheel is unavailable. Do not install conflicting OpenCV
packages together. Caffe MobileNet-SSD requires **OpenCV 4.x**, not 5.x.
All new modules import safely without these optional packages.

## Camera and notebook stream

```python
from rasptank import RaspTank

# Starts stopped motors; camera is configured but not started.
tank = RaspTank(enable_camera=True, command_timeout=0.5)
tank.start_camera()
view = tank.show_camera(fps=10)  # nonblocking; replaces the tank's previous view
```

Separate notebook cells may issue `tank.forward(15)`, `tank.turn_left(15)` or
`tank.stop()` while video updates. Every manual command expires after
`command_timeout` seconds unless renewed. Low duty may stall rather than move.
`tank.stop()` cancels autonomous output before stopping; direct
`tank.motors.*` calls bypass arbitration and the watchdog.

For camera-only work without opening motors:

```python
from rasptank import CameraController
from rasptank.notebook import show_camera

camera = CameraController(size=(640, 480), fps=15, max_age=0.5)
camera.start()
view = show_camera(camera, previous=globals().get("view"))
frame = camera.capture(timeout=1)
jpeg = camera.snapshot()
```

`Frame` holds immutable packed **RGB** bytes, JPEG bytes, dimensions, sequence
and a monotonic timestamp taken before the capture request. Capture flushes
queued exposures so timestamps cannot disguise cached pre-stop frames as fresh.
Picamera2's Pillow
image conversion determines channel order; confirm it with a physical color
target. One capture process and one latest-frame slot serve all consumers.
Slow displays drop intermediate frames. `capture(after_sequence=...,
min_timestamp=...)` can require a newer, post-stop frame; stale images raise
`CameraTimeoutError`. No reading fabricates a disabled camera image.

`start()` and normal `stop()` are repeatable; `close()` permanently closes.
Camera startup/frame waits are bounded. A stuck driver lives in a spawned
process: shutdown first requests cleanup, then terminates/kills if needed.
The maximum shutdown wait is three times `shutdown_timeout` for process joins,
plus one `shutdown_timeout` for the reader. Forced termination relies on OS
device release rather than pretending driver cleanup completed. Only the CSI
Picamera2 backend is provided; a USB camera needs an injected source factory.
Factories must be spawn-picklable importable definitions, not notebook lambdas.

The display borrows the camera; stopping/closing a view does not close capture.
`Stop video` is a presentation control, **not** an emergency motor stop.
Use `tank.stop()` for motion. Views show errors in their status widget;
`view.status()` also reports them. Tank-attached view errors stop supervised
motion. A custom annotation transform must return nonempty JPEG bytes.
Stuck transforms are reported on bounded close, not silently joined forever.

The motor watchdog remains active independently of display updates. It does not
rely on browser connection detection: a lost browser cannot renew expired
manual commands. A read-only viewer disconnect need not end a finite roaming
session; remain physically present during every autonomous trial.

Always clean up, stopping supervised motion first:

```python
try:
    # preview/control work
    pass
finally:
    tank.close()  # closes attached views and owned controllers
```

If a standalone camera/view is used, close the view and camera separately.
Injected camera/motor/sensor controllers are borrowed, not closed by the tank.
Supervision stops borrowed motors on cancellation/close but does not release
their device ownership.

## Specific objects

`detect_color(frame, hsv_ranges=hsv_target(0), min_area=100, label="red")`
returns color contours, with pixel area and **no invented model confidence**.
`hsv_target` handles hue wrap at red. Color is not semantic identity.
`annotate(frame, detections)` returns a new JPEG without modifying the source.

`MobileNetSSD` is an explicit local Caffe adapter for VOC's 20 object classes:
aircraft, bicycle, bird, boat, bottle, bus, car, cat, chair, cow, dining table,
dog, horse, motorbike, person, potted plant, sheep, sofa, train and TV monitor.
Use exact identifiers from `rasptank.vision.VOC_LABELS` (for example
`aeroplane`, `diningtable`, `pottedplant`, `tvmonitor`). It cannot identify one
personal item among several examples of the same class.

### Optional model setup (explicit downloads only)

The tested assets come from
[`chuanqi305/MobileNet-SSD`](https://github.com/chuanqi305/MobileNet-SSD)
at commit `bb17b6c3eef36d80be441ae8e5339be66e8e3b7a`. Its repository declares
the MIT license; retain its license/attribution with downloaded assets and
review upstream model/training-data terms for your use. No assets are bundled
or fetched by the Python API.

From the repository root, after reviewing the upstream source:

```sh
mkdir -p models
curl --fail --location https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/bb17b6c3eef36d80be441ae8e5339be66e8e3b7a/deploy.prototxt -o models/deploy.prototxt
curl --fail --location https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/bb17b6c3eef36d80be441ae8e5339be66e8e3b7a/mobilenet_iter_73000.caffemodel -o models/mobilenet_iter_73000.caffemodel
curl --fail --location https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/bb17b6c3eef36d80be441ae8e5339be66e8e3b7a/LICENSE -o models/LICENSE
sha256sum models/deploy.prototxt models/mobilenet_iter_73000.caffemodel
```

Expected SHA256, also exported as `MOBILENET_PROTOTXT_SHA256` and
`MOBILENET_WEIGHTS_SHA256`:

```text
2d180f723b3109e21f8287f6b3c691390d07b60eed998327cd3259ffa0e50608  deploy.prototxt
52eed8be80522c152a17fb56740de705b79881bde1a167e0e747310523685fc7  mobilenet_iter_73000.caffemodel
```

```python
from rasptank.vision import (
    MobileNetSSD, MOBILENET_PROTOTXT_SHA256, MOBILENET_WEIGHTS_SHA256,
    detect_objects, annotate, find_object,
)

detector = MobileNetSSD(
    "models/deploy.prototxt", "models/mobilenet_iter_73000.caffemodel",
    prototxt_sha256=MOBILENET_PROTOTXT_SHA256,
    weights_sha256=MOBILENET_WEIGHTS_SHA256,
)
frame = camera.capture()
detections = detect_objects(frame, detector=detector, labels=["bottle"], confidence=0.5)
annotated_jpeg = annotate(frame, detections)
matches = find_object(camera, detector=detector, labels=["bottle"], timeout=5)
```

Detections contain label, confidence, pixel `(x,y,width,height)`, normalized
center, frame sequence/timestamp and optional contour area. Inject an adapter
with `detect(frame, *, labels, confidence)` returning these `Detection` values
for a custom model or instance recognition. Detection/annotation never commands
motors, lights or servos.

`detect_objects` is synchronous. `find_object` runs a spawn-picklable detector
in an isolated bounded process, including model loading in its deadline.
An unfinished inference raises `TimeoutError`; completed no-match searches
return `[]`. Search ends at deadline or `max_frames` (default 100); camera/model
errors raise. Cleanup can add up to 1 s beyond the search timeout.
Each search reloads MobileNet assets in the child, so benchmark overhead.
Start with snapshot detection; measure sustained inference and preview rates
on your Pi before attempting annotated live video.

## Room roaming

Roaming uses **both** fresh ultrasonic readings and camera observations.
Named-object detection is not a free-space detector. The provided
`FloorObserver` is a restrictive, calibrated HSV check of a lower-center
forward floor corridor, only useful on a uniform illuminated flat trial floor.
It returns clear, blocked or unknown. Similar-colored obstacles, reflections,
glass and drops remain blind spots: it cannot establish collision-free travel.

Before enabling motion, measure floor HSV ranges from stationary real images,
set the ROI to the actual forward corridor, verify ultrasonic/camera alignment,
and test clear floor, clutter and low light. Ranges use H 0..179, S/V 0..255;
split hue-wrapping targets into intervals. Darkness/occlusion is unknown and
stops motion. Broad ranges that include furniture defeat this heuristic.

```python
from rasptank import RaspTank, RoamPolicy
from rasptank.vision import FloorObserver

# measured_floor_ranges must come from the actual floor, not copied defaults.
observer = FloorObserver(measured_floor_ranges, roi=(0.3, 0.55, 0.7, 0.95))
tank = RaspTank(camera=camera, enable_ultrasonic=True, command_timeout=0.5)
policy = RoamPolicy(
    speed=20, stop_distance=0.5, resume_distance=0.6,
    max_age=0.3, pulse=0.1, max_turns=12, max_duration=10,
)
session = tank.start_roaming(observer=observer, policy=policy)
# Other cells: tank.roaming_status(), tank.stop(), session.stop()
```

Those numeric values are initial software settings, **not validated physical
limits**. A low-speed pulse is followed by a complete stop and fresh post-stop
observations before the next action. Close range always vetoes forward;
visual blockage permits only short left turns; unknown visibility stops.
Hysteresis and finite turn recovery prevent endless oscillation. Duration is
finite, and there is no unobserved backward travel or automatic servo centering.
At/below `emergency_distance` (initial 0.2 m), the policy stops rather than
turning with insufficient clearance; this threshold also requires measurement.
This initial policy uses fixed camera/range alignment, not radar sweeps or SLAM.

Invalid, future, stale or failed observations fault the session. An independent
watchdog stops an expired motor command or stalled assessment without waiting
for the perception thread. A Stop/manual-override token prevents an old
perception result from restarting motion. A manual facade command cancels
roaming first. Repeat starts cannot create competing workers.
Faults latch; inspect `tank.roaming_status()["error"]`, then explicitly
`tank.reset_drive_fault()` while stopped. A hung custom observer is a daemon,
reported on bounded shutdown and prevents another session while it is alive;
do not reuse its borrowed devices until it exits. Status preserves motor-stop
failures rather than claiming successful shutdown.

`command_timeout=None` preserves the existing unsupervised synchronous API.
Roaming refuses that mode. Watchdog checks run every 10 ms or less, but actual
stop latency depends on Python scheduling and bounded GPIO operations.
**Process/OS failure or a hung GPIO call can defeat software supervision.**
Keep a physical cutoff; never leave the tank roaming unattended.

## Troubleshooting: "Camera(s) not found"

If `camera.start()` reports `libcamera detected no camera` (Picamera2 logs
`Camera(s) not found` / `Camera __init__ sequence did not complete`), libcamera
enumerated zero cameras. The tank code has not touched the camera yet; fix the
Pi first. Close the notebook camera cell, then in a Pi terminal:

```sh
libcamera-hello --list-cameras      # Bullseye; Bookworm: rpicam-hello --list-cameras
vcgencmd get_camera                 # legacy stack status; detected=0 confirms no sensor
CONFIG=/boot/config.txt; [ -f /boot/firmware/config.txt ] && CONFIG=/boot/firmware/config.txt
grep -nE '^\s*(start_x|camera_auto_detect|dtoverlay|gpu_mem)' "$CONFIG"
dmesg | grep -iE 'ov5647|imx219|imx477|imx708|unicam|cfe'
```

Fix according to what you find, then reboot:

1. **Legacy camera enabled** (`start_x=1`, or raspi-config shows Legacy Camera
   enabled). Older Adeept instructions enabled it; libcamera/Picamera2 cannot
   use the camera then. Run `sudo raspi-config` -> Interface Options ->
   Legacy Camera -> **Disable**, or comment out `start_x=1`.
2. **Auto-detect off**: ensure `camera_auto_detect=1` under `[all]`.
3. **Auto-detect does not find a third-party module** (common with OV5647
   "Pi Camera v1"-compatible boards): set `camera_auto_detect=0` and add the
   matching overlay, e.g. `dtoverlay=ov5647` (v1), `imx219` (v2), `imx477`
   (HQ) or `imx708` (v3). On a Pi 5/CM4 second port add `,cam0` if required.
4. **Cable/connection**: power off, reseat the ribbon at both ends with the
   contacts facing the correct way (Pi 5 needs the narrow 22-pin cable).

Example `config.txt` edit (back it up first; use the overlay for *your* sensor):

```sh
sudo cp "$CONFIG" "$CONFIG.bak"
sudo sed -i 's/^\s*start_x=1/#start_x=1/; s/^camera_auto_detect=0/camera_auto_detect=1/' "$CONFIG"
grep -q '^camera_auto_detect=' "$CONFIG" || echo 'camera_auto_detect=1' | sudo tee -a "$CONFIG"
tail -n 15 "$CONFIG"   # appended lines must fall under [all], not e.g. [pi4]/[cm4]
sudo reboot
```

Only if `--list-cameras` still shows nothing with auto-detect, switch to an
explicit overlay:

```sh
sudo sed -i 's/^camera_auto_detect=1/camera_auto_detect=0/' "$CONFIG"
echo 'dtoverlay=ov5647' | sudo tee -a "$CONFIG"   # replace with your sensor
sudo reboot
```

Continue only when `--list-cameras` lists a sensor and `libcamera-hello -t 2000`
(or `rpicam-hello`) shows a preview. If the camera is listed but start fails
with "busy", stop the legacy `webServer.py`/autorun service or any other
camera user. A notebook kernel that already failed can be reused once the
camera is detected; rerun the camera cell.

## Validation and physical acceptance

Run `python -m unittest discover -s tests`; vision/widget tests need the optional
packages and otherwise skip explicitly. Camera/process tests use synthetic
sources, policy tests use exact injected-clock thresholds, and all notebooks
default to hardware-free execution.
Widget worker checks are software-only; actual Jupyter/VS Code frontend
rendering and Pi throughput remain physical/environment acceptance work.

To include the optional actual model loading/inference/spawned-search check,
after manually installing the verified assets:

```sh
RASPTANK_TEST_MODEL_DIR="$PWD/models" python -m unittest discover -s tests -p test_vision.py
```

Use `notebooks/rasptank_camera_roaming.ipynb` for separately gated camera,
color/semantic detection, manual motion, floor checks and finite roaming.
First verify camera colors/latency while motors are disabled. Then test the
watchdog and Stop/manual override on a stand. Measure stopping travel and
worst-case frame age under simultaneous preview/inference before setting
clearance, speed, sensor-age and command-lease budgets. Only then perform
short supervised floor trials with a cutoff, barriers and no stairs or people.
The package does not claim those physical acceptance gates have passed.
