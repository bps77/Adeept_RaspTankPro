# Notebook camera, object identification and room roaming

Status: proposed implementation plan; no camera or autonomy code is added yet.
Branch: `feature/notebook-camera-autonomy`, based on `master`.

## Goals and boundaries

1. Stream live video inside a notebook while separate cells control the tank.
2. Identify selected objects and show annotated detections without moving motors.
3. Provide explicit, supervised room-roaming methods using both the ultrasonic
   range finder and camera, with immediate manual override.

The initial target is a Python kernel running on the Pi, including notebooks
accessed through Remote SSH or an SSH-tunneled Jupyter session. A laptop kernel
controlling a remote Pi would require a separate authenticated control protocol
and is not included. No internet-facing camera/control server is required.
This is reactive obstacle avoidance, not SLAM, guaranteed collision avoidance,
or unattended navigation. Stairs, glass, soft surfaces, blind spots, pets and
people remain hazards; trials require supervision and a physical power cutoff.

## Existing code to build on

* `rasptank/tank.py`: optional controllers, owned/borrowed lifecycle and
  construction rollback. Add camera integration without changing defaults.
* `rasptank/hardware/_lifecycle.py`: closed-state checks and best-effort cleanup.
* `rasptank/behaviors.py`: bounded steps, injectable timing and stop-on-failure.
  Keep existing `obstacle_step` behavior intact; add a separate fused policy.
* `server/camera_opencv.py`: Picamera2 capture and HSV/color tracking are useful
  references, but its imports/class bodies initialize hardware and its vision
  changes motor/servo/light outputs. Do not import it into the package.
* `server/base_camera.py`: streaming reference only; replace its shared global
  state and unbounded frame waits with instance-owned, bounded operations.
* Existing notebooks and `tests/`: preserve hardware-disabled defaults,
  standard-library `unittest` mocks and notebook JSON/Python syntax checks.

## Proposed API and architecture

Names below are proposed contracts, not currently available methods.

### 1. Camera wrapper

Add `rasptank/hardware/camera.py` with `CameraController` and facade options
`RaspTank(enable_camera=False, camera=None)`.

* Lazily load Picamera2/libcamera only when camera hardware is explicitly enabled.
  Start with the CSI/Picamera2 backend used by the legacy server; defer a USB
  backend unless the installed camera requires it.
* Expose `start()`, `capture(timeout=...)`, `snapshot(timeout=...)`, `stop()` and
  `close()`. Define resolution, frame-rate limit, flips and JPEG quality with
  validation before opening/configuring hardware.
* Define frame results containing sequence number, monotonic capture timestamp,
  dimensions, pixel format and image data. Normalize one documented color
  order at the backend boundary; verify against a physical color target rather
  than assuming a Picamera2 format name implies NumPy channel order.
* Use a single owned capture worker and a latest-frame buffer of capacity one.
  Display and detection share frames, never open a second camera; slow consumers
  drop old frames instead of accumulating latency. Specify copy/read-only
  ownership so overlays cannot mutate frames another consumer is using.
* Bound startup, frame waits and shutdown. Propagate capture failures explicitly;
  never return a stale frame as if it were fresh. Investigate Picamera2 request
  cancellation first; if a driver call cannot be bounded/cancelled, use an
  isolated capture process rather than claiming a timed thread join releases it.
* Disabled mode never opens a device or fabricates images. Injected frame
  sources provide deterministic tests. Borrowed camera controllers stay
  caller-owned, matching the existing facade.

### 2. Notebook video and controls

Add `rasptank/notebook.py` as an optional presentation layer.

* `show_camera(camera, fps=...)` returns a stream handle with `stop()`, `close()`
  and visible running/error status. Use an `ipywidgets.Image` JPEG view and a
  rate-limited update worker so the starting cell returns immediately.
* Keep IPython/ipywidgets imports lazy and report missing dependencies with
  installation instructions. Verify updates in both Jupyter and VS Code
  Remote SSH notebooks; keep widget/event-loop details out of camera capture.
* Separate cells can call `tank.forward(...)`, turns and `tank.stop()` while
  the view updates. An optional control panel can reuse the same command path,
  including a prominent Stop button, rather than directly writing GPIO.
* Re-running a stream cell must close its previous handle. Closing the display
  must not close a borrowed camera. Tank shutdown must invalidate attached
  consumers and terminate its owned workers without leaking threads.
* Notebook driving requires an opt-in supervised command timeout. A forgotten
  manual drive command must expire even when no autonomy loop is running.
  On widget/control disconnect, stop motion; losing a read-only viewer is not
  a substitute for a motor watchdog.

### 3. Object identification helpers

Add `rasptank/vision.py` with motor-independent detection and annotation.

* `detect_color(frame, hsv_ranges=..., min_area=...)` supplies a lightweight
  baseline, including hue wrap-around. Color matches are not semantic identity.
* `detect_objects(frame, labels=..., confidence=...)` uses an injected detector
  contract and one explicitly selected lightweight, Pi-compatible local model.
  Choose its runtime after measuring the Pi; do not silently download weights.
  Document model license, provenance, checksum, class coverage and setup.
* Return structured detections: label, optional model confidence, pixel bounding
  box, normalized center and source frame sequence/timestamp. Color results
  expose area/match metadata, not invented classification confidence.
* Provide `annotate(frame, detections)` and a bounded `find_object(...)` helper
  with explicit no-match/timeout results and camera/model errors that raise.
  Identification alone never drives tracks or pans servos.
* Support selecting known model labels first. Recognizing a particular personal
  item needs a separate reference-image/custom-model decision; class detection
  does not distinguish one specific instance from another.

### 4. Fused roaming policy and supervised runner

Add a bounded `roam_step(...)` policy alongside existing behaviors, then a
separate opt-in supervisor (proposed `rasptank/autonomy.py`).

* Use states such as stopped, assessing, advancing, scanning, turning, blocked
  and fault. Each step returns action, reason, sensor timestamps and relevant
  readings for notebook telemetry. Validate finite ranges, thresholds, speeds,
  durations and confidence/age limits before any motor output.
* Require fresh, valid ultrasonic and camera observations before moving.
  Close ultrasonic returns always veto forward motion. Camera assessment adds
  a conservative obstacle/unknown veto; no detections alone do not prove a
  clear path, and monocular bounding boxes are not metric distances.
* Define and evaluate the visual obstacle/free-space assessment separately from
  named-object detection using recorded room scenes. If it cannot establish
  usable visibility, stop rather than silently falling back to ultrasonic-only
  roaming. Treat low light, occlusion and unsupported scenes as unknown.
* Approach cautiously using configurable, physically validated speed and stop
  distance. Near obstacles, stop first, optionally scan ultrasonic directions,
  then perform short bounded turns and remeasure. Use hysteresis to avoid
  turn oscillation and a finite recovery budget; a trapped robot stays stopped.
  Do not automatically reverse into an unobserved rear area.
* Radar scan requires explicit servo enablement and verified channel 1
  calibration. Camera pan/tilt channels 0/4 are legacy references only: verify
  linkage and coordinate alignment on the actual tank. No implicit centering.
  Any scan invalidates forward observations until alignment is restored and
  fresh frames/ranges are acquired.
* Proposed facade methods: `start_roaming(...)`, `stop_roaming()` and
  `roaming_status()`. Start returns a supervised session handle immediately;
  repeat starts cannot create competing drivetrain owners.
* Centralize command arbitration: manual motion cancels roaming before taking
  control; `tank.stop()` cancels autonomous output and stops tracks. Serialize
  transitions so a queued autonomy command cannot restart motion after Stop.
  Document that direct low-level motor calls bypass facade arbitration.
* Add an independent motor command-expiry watchdog, finite actuation leases,
  bounded worker shutdown and fault latching. Camera hangs, range errors,
  inference stalls, cancellation and notebook interruption must stop motion
  without waiting for the vision worker. A worker may resume only after an
  explicit restart. Stop borrowed motors on session cancellation but do not
  close caller-owned devices.
* Software supervision cannot stop motors after complete process/OS failure.
  Document this limitation and require a physical cutoff; do not promise a
  thread watchdog is a hardware emergency stop.

## Delivery sequence and acceptance gates

| Phase | Deliverables | Gate before proceeding |
|---|---|---|
| 1. Capture | Camera controller, facade exports/ownership, optional dependency instructions, mock tests | Inert imports; disabled access raises; bounded failures; owned/borrowed cleanup and repeated start/stop verified |
| 2. Notebook view | Nonblocking stream handle and hardware-disabled camera notebook | Other cells remain usable while streaming; stable single-frame buffering; reruns, errors and close do not leak workers |
| 3. Vision | Color helper, detector adapter, annotations and object examples | Synthetic/recorded fixtures verify boxes, filtering, hue wrap, no-match and timestamps; chosen model benchmarked on the Pi |
| 4. Driving supervision | Command arbitration, manual expiry watchdog and session lifecycle | Deterministic race/stall tests prove Stop cannot be undone and expired commands stop independently of capture/inference |
| 5. Fused roaming | Bounded policy, telemetry and opt-in notebook examples | Both sensors influence decisions; either invalid/stale input stops; near range always vetoes forward; turns/recovery are bounded |
| 6. Physical trials | Camera-only, stand-supported control, then supervised floor trials | Measure frame age, inference latency, sensor alignment, stopping travel and watchdog latency before enabling room roaming |

For phases 4-6, define a numeric maximum command lease and sensor-age budget
from measured Pi latency and stopping travel. Software tests use an injected
clock to assert the exact configured thresholds and motor call sequence; Pi
trials measure actual timing, which thread scheduling cannot strictly guarantee.
Target a configurable 640x480 preview initially, but benchmark sustained preview
and inference together before selecting default rates or a semantic model.

Update README.md, docs/FEATURES.md, AGENT_MEMORY.md, exports and contract
docstrings with each delivered phase. Add a dedicated notebook with separate
camera, vision and motion opt-ins; default execution must remain hardware-free.
Keep legacy servers unchanged and never run them concurrently with new owners.

## Decisions to resolve before the affected implementation

* Pi model, OS/Python environment and actual camera interface.
* Whether "specific objects" means known classes, color targets, or individual
  items; desired target labels and available reference images.
* Acceptable preview rate and semantic inference rate on that Pi.
* Camera/range-finder alignment, safe servo pulses, usable low motor duty,
  stopping distance and permitted trial environment.
* Model/runtime choice and dependency setup after the hardware benchmark.

The reported successful notebook runs are useful initial feedback, not evidence
that camera streaming, fused roaming or the new watchdog has been validated.
