"""Optional local OpenCV vision; detection never commands motors or servos."""

from dataclasses import dataclass
from pathlib import Path
import hashlib
import multiprocessing
import queue
import time
from typing import Optional

from .hardware._lifecycle import number

VOC_LABELS = ("background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",
              "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike",
              "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor")
MOBILENET_COMMIT = "bb17b6c3eef36d80be441ae8e5339be66e8e3b7a"
MOBILENET_PROTOTXT_SHA256 = "2d180f723b3109e21f8287f6b3c691390d07b60eed998327cd3259ffa0e50608"
MOBILENET_WEIGHTS_SHA256 = "52eed8be80522c152a17fb56740de705b79881bde1a167e0e747310523685fc7"


def _modules():
    """Load optional dependencies on use, with actionable missing-package errors."""
    try:
        import cv2
        import numpy as np
    except ImportError as exc:
        raise ImportError("Vision requires numpy and opencv; see requirements-camera.txt") from exc
    return cv2, np


def _image(frame):
    """Return a read-only RGB ndarray view over immutable frame bytes."""
    _, np = _modules()
    return np.frombuffer(frame.rgb, dtype=np.uint8).reshape(frame.height, frame.width, 3)


def _ranges(hsv_ranges):
    """Validate nonwrapping OpenCV HSV intervals (H 0..179, S/V 0..255)."""
    result = []
    for lower, upper in hsv_ranges:
        if len(lower) != 3 or len(upper) != 3:
            raise ValueError("HSV bounds must each have three integers")
        for index, high in enumerate((179, 255, 255)):
            if (type(lower[index]) is not int or type(upper[index]) is not int
                    or not 0 <= lower[index] <= upper[index] <= high):
                raise ValueError("HSV ranges must be ordered within H 0..179, S/V 0..255")
        result.append((tuple(lower), tuple(upper)))
    if not result:
        raise ValueError("At least one HSV range is required")
    return tuple(result)


def hsv_target(hue, saturation=100, value=100, tolerance=15):
    """Build color intervals around hue, handling wrap at 179; minimum S/V thresholds."""
    for name, val, high in (("hue", hue, 179), ("saturation", saturation, 255),
                            ("value", value, 255), ("tolerance", tolerance, 89)):
        if type(val) is not int or not 0 <= val <= high:
            raise ValueError(f"{name} must be an integer in 0..{high}")
    low, high = hue - tolerance, hue + tolerance
    intervals = [(max(0, low), min(179, high))]
    if low < 0:
        intervals.append((180 + low, 179))
    if high > 179:
        intervals.append((0, high - 180))
    return tuple(((a, saturation, value), (b, 255, 255)) for a, b in intervals)


def _mask(frame, hsv_ranges):
    """Return HSV and union mask for validated intervals; use real RGB conversion."""
    cv2, np = _modules()
    hsv = cv2.cvtColor(_image(frame), cv2.COLOR_RGB2HSV)
    mask = np.zeros((frame.height, frame.width), dtype=np.uint8)
    for lower, upper in _ranges(hsv_ranges):
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, np.array(lower), np.array(upper)))
    return hsv, mask


@dataclass(frozen=True)
class Detection:
    """Label, optional model confidence, pixel xywh box, normalized center and frame ID."""

    label: str
    confidence: Optional[float]
    box: tuple
    center: tuple
    sequence: int
    timestamp: float
    area: float = 0

    def __post_init__(self):
        """Validate metadata and positive pixel box; image-specific bounds are checked later."""
        if not isinstance(self.label, str) or not self.label:
            raise ValueError("label must be nonempty text")
        if self.confidence is not None:
            number(self.confidence, 0, 1, "confidence")
        if (len(self.box) != 4 or any(type(v) is not int for v in self.box)
                or self.box[0] < 0 or self.box[1] < 0 or min(self.box[2:]) <= 0):
            raise ValueError("box must be nonnegative xy and positive integer width/height")
        if len(self.center) != 2:
            raise ValueError("center must contain two normalized coordinates")
        for value in self.center:
            number(value, 0, 1, "center")
        if type(self.sequence) is not int or self.sequence < 1:
            raise ValueError("sequence must be a positive integer")
        number(self.timestamp, 0, float("inf"), "timestamp")
        number(self.area, 0, float("inf"), "area")


def _detection(frame, label, confidence, box, area=0):
    """Construct a clipped positive pixel box and its normalized center."""
    x, y, width, height = box
    x, y = max(0, int(x)), max(0, int(y))
    width, height = min(int(width), frame.width - x), min(int(height), frame.height - y)
    if width <= 0 or height <= 0:
        raise ValueError("Detection box must overlap the image")
    return Detection(label, confidence, (x, y, width, height),
                     ((x + width / 2) / frame.width, (y + height / 2) / frame.height),
                     frame.sequence, frame.timestamp, float(area))


def detect_color(frame, *, hsv_ranges, min_area=100, label="color"):
    """Return contour matches, not semantic identity; confidence is None, area is pixels."""
    number(min_area, 1, frame.width * frame.height, "min_area")
    if not isinstance(label, str) or not label:
        raise ValueError("label must be a nonempty string")
    cv2, _ = _modules()
    if not hasattr(cv2.dnn, "readNetFromCaffe"):
        raise ImportError("MobileNetSSD requires OpenCV 4.x with Caffe DNN support")
    _, mask = _mask(frame, hsv_ranges)
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    return [_detection(frame, label, None, cv2.boundingRect(c), cv2.contourArea(c))
            for c in contours if cv2.contourArea(c) >= min_area]


class MobileNetSSD:
    """Explicit local Caffe MobileNet-SSD/VOC adapter; no network access/downloads.

    Use compatible chuanqi305/MobileNet-SSD deploy.prototxt and model weights,
    independently verify their provenance/license, and supply SHA256 digests.
    Instances are synchronous; a single caller owns inference.
    """

    def __init__(self, prototxt, weights, *, prototxt_sha256, weights_sha256):
        """Verify both local assets before loading the optional OpenCV DNN runtime."""
        for path, expected in ((prototxt, prototxt_sha256), (weights, weights_sha256)):
            if (not isinstance(expected, str) or len(expected) != 64
                    or any(c not in "0123456789abcdef" for c in expected)):
                raise ValueError("SHA256 must be 64 lowercase hexadecimal characters")
            if hashlib.sha256(Path(path).read_bytes()).hexdigest() != expected:
                raise ValueError(f"SHA256 mismatch: {path}")
        cv2, _ = _modules()
        self._net = cv2.dnn.readNetFromCaffe(str(prototxt), str(weights))
        self._assets = dict(prototxt=str(Path(prototxt).resolve()),
                            weights=str(Path(weights).resolve()),
                            prototxt_sha256=prototxt_sha256, weights_sha256=weights_sha256)

    def __getstate__(self):
        """Serialize verified asset paths, never a native OpenCV network object."""
        return self._assets

    def __setstate__(self, assets):
        """Reverify/reload assets inside a spawned bounded-search worker."""
        self.__init__(**assets)

    def detect(self, frame, *, labels=None, confidence=0.5):
        """Infer VOC classes; return filtered detections; invalid model output raises."""
        number(confidence, 0, 1, "confidence")
        selected = None if labels is None else set(labels)
        if selected is not None and (not selected or not selected <= set(VOC_LABELS[1:])):
            raise ValueError("labels must be nonempty known VOC classes")
        cv2, np = _modules()
        bgr = cv2.cvtColor(_image(frame), cv2.COLOR_RGB2BGR)
        blob = cv2.dnn.blobFromImage(bgr, 0.007843, (300, 300), 127.5, swapRB=False)
        self._net.setInput(blob)
        output = self._net.forward()
        if output.ndim != 4 or output.shape[:2] != (1, 1) or output.shape[3] != 7:
            raise ValueError("Expected MobileNet-SSD output shaped (1,1,N,7)")
        result = []
        for row in output[0, 0]:
            if not np.isfinite(row).all():
                raise ValueError("Model returned nonfinite detection")
            class_id = int(row[1])
            if row[1] != class_id or not 0 <= class_id < len(VOC_LABELS):
                raise ValueError("Model returned invalid class ID")
            score = float(number(float(row[2]), 0, 1, "model confidence"))
            label = VOC_LABELS[class_id]
            if class_id == 0 or score < confidence or (selected is not None and label not in selected):
                continue
            x1, y1, x2, y2 = np.clip(row[3:7], 0, 1)
            result.append(_detection(frame, label, score,
                                    (int(x1 * frame.width), int(y1 * frame.height),
                                     int((x2 - x1) * frame.width),
                                     int((y2 - y1) * frame.height))))
        return result


def detect_objects(frame, *, detector, labels=None, confidence=0.5):
    """Call an explicit detector.detect adapter; never assume model weights exist."""
    number(confidence, 0, 1, "confidence")
    detections = detector.detect(frame, labels=labels, confidence=confidence)
    if not isinstance(detections, (list, tuple)):
        raise ValueError("detector.detect must return a list/tuple of Detection")
    for detection in detections:
        if not isinstance(detection, Detection):
            raise ValueError("detector.detect must return Detection values")
        x, y, width, height = detection.box
        if (detection.sequence != frame.sequence or detection.timestamp != frame.timestamp
                or x + width > frame.width or y + height > frame.height):
            raise ValueError("Detector returned mismatched frame metadata or out-of-bounds box")
    return list(detections)


def annotate(frame, detections):
    """Return a new annotated JPEG; reject detections from a different frame."""
    cv2, _ = _modules()
    image = cv2.cvtColor(_image(frame), cv2.COLOR_RGB2BGR)
    for detection in detections:
        if detection.sequence != frame.sequence or detection.timestamp != frame.timestamp:
            raise ValueError("Detection and image must share a frame")
        x, y, width, height = detection.box
        cv2.rectangle(image, (x, y), (x + width - 1, y + height - 1), (0, 255, 0), 2)
        text = detection.label
        if detection.confidence is not None:
            text += f" {detection.confidence:.2f}"
        cv2.putText(image, text, (x, max(15, y)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 1)
    success, encoded = cv2.imencode(".jpg", image)
    if not success:
        raise RuntimeError("Could not encode annotated JPEG")
    return encoded.tobytes()


def _search_process(detector, frames, results, labels, confidence):
    """Run potentially blocking inference in an isolated, parent-terminable process."""
    try:
        while True:
            frame = frames.get()
            if frame is None:
                return
            detections = detect_objects(frame, detector=detector, labels=labels,
                                        confidence=confidence)
            results.put(("result", detections))
    except BaseException as exc:
        results.put(("error", f"{type(exc).__name__}: {exc}"))


def find_object(camera, *, detector, labels, timeout=5, confidence=0.5, max_frames=100):
    """Search fresh frames in a bounded process; [] means completed inferences found no match.

    Detector must be spawn-picklable (MobileNetSSD serializes its verified assets).
    Camera/model errors raise. Camera.capture must honor its timeout contract.
    Timeout includes process/model startup; cleanup adds at most 1 s for joins.
    Search ends at max_frames or deadline; an unfinished inference at deadline
    raises TimeoutError rather than claiming no match.
    No model inference thread is left running after this helper returns.
    """
    number(timeout, 0.01, 3600, "timeout")
    number(confidence, 0, 1, "confidence")
    if type(max_frames) is not int or not 1 <= max_frames <= 10000:
        raise ValueError("max_frames must be an integer in 1..10000")
    context = multiprocessing.get_context("spawn")
    frames, results = context.Queue(maxsize=1), context.Queue(maxsize=1)
    process = context.Process(target=_search_process,
                              args=(detector, frames, results, labels, confidence), daemon=True)
    deadline, sequence = time.monotonic() + timeout, None
    try:
        process.start()
        for _ in range(max_frames):
            if time.monotonic() >= deadline:
                break
            frame = camera.capture(timeout=min(1, max(0.001, deadline - time.monotonic())),
                                   after_sequence=sequence)
            sequence = frame.sequence
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            frames.put(frame, timeout=remaining)
            while time.monotonic() < deadline:
                try:
                    kind, payload = results.get(timeout=min(0.05, max(
                        0.001, deadline - time.monotonic())))
                except queue.Empty:
                    if not process.is_alive():
                        raise RuntimeError(f"Object search process exited ({process.exitcode})")
                    continue
                if kind == "error":
                    raise RuntimeError(f"Object detector failed: {payload}")
                if time.monotonic() >= deadline:
                    raise TimeoutError("Object inference exceeded search deadline")
                if payload:
                    return payload
                break
            else:
                raise TimeoutError("Object inference did not finish before search deadline")
        return []
    finally:
        if process.pid is not None:
            process.terminate()
            process.join(0.5)
            if process.is_alive():
                process.kill()
                process.join(0.5)
            if process.is_alive():
                raise RuntimeError("Object search process did not stop after kill")
        process.close()
        for channel in (frames, results):
            channel.cancel_join_thread()
            channel.close()


@dataclass(frozen=True)
class VisualObservation:
    """Explicit clear/blocked/unknown corridor verdict with originating frame identity."""

    state: str
    sequence: int
    timestamp: float
    floor_fraction: float


class FloorObserver:
    """Conservative, explicitly calibrated HSV floor-corridor check, not depth sensing.

    Only appropriate on a uniform, flat, enclosed trial floor. Similar-colored
    hazards and drops cannot be detected reliably; no observation proves safety.
    Missing/poorly lit pixels yield unknown, never clear by default.
    """

    def __init__(self, hsv_ranges, *, roi=(0.3, 0.55, 0.7, 0.95),
                 clear_fraction=0.98, min_value=40, min_bright_fraction=0.95):
        """Configure normalized xyxy corridor and required floor/illumination fractions."""
        self.ranges = _ranges(hsv_ranges)
        if len(roi) != 4:
            raise ValueError("roi must contain x1,y1,x2,y2")
        self.roi = tuple(number(v, 0, 1, "roi") for v in roi)
        if self.roi[0] >= self.roi[2] or self.roi[1] >= self.roi[3]:
            raise ValueError("roi must have positive area")
        self.clear_fraction = number(clear_fraction, 0.9, 1, "clear_fraction")
        self.min_bright_fraction = number(min_bright_fraction, 0.9, 1, "min_bright_fraction")
        self.min_value = number(min_value, 1, 255, "min_value")

    def __call__(self, frame):
        """Assess calibrated corridor, returning unknown for darkness/empty crops."""
        _, np = _modules()
        hsv, mask = _mask(frame, self.ranges)
        x1, y1, x2, y2 = self.roi
        region = (slice(int(y1 * frame.height), int(y2 * frame.height)),
                  slice(int(x1 * frame.width), int(x2 * frame.width)))
        pixels = mask[region]
        if not pixels.size:
            return VisualObservation("unknown", frame.sequence, frame.timestamp, 0)
        fraction = float(np.count_nonzero(pixels) / pixels.size)
        bright = float(np.count_nonzero(hsv[region][:, :, 2] >= self.min_value) / pixels.size)
        state = ("unknown" if bright < self.min_bright_fraction else
                 "clear" if fraction >= self.clear_fraction else "blocked")
        return VisualObservation(state, frame.sequence, frame.timestamp, fraction)
