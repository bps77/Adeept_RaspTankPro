"""Synthetic real-OpenCV fixtures, with no camera/model download or motor access."""

import hashlib
import os
import tempfile
from pathlib import Path
import time
import unittest
from unittest.mock import Mock, patch

from rasptank import Frame
from rasptank.vision import (FloorObserver, MobileNetSSD, annotate, detect_color,
                            detect_objects, find_object, hsv_target, Detection,
                            MOBILENET_PROTOTXT_SHA256, MOBILENET_WEIGHTS_SHA256)

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None


class SearchDetector:
    """Spawn-picklable adapter for search success, no-match, fault and hang tests."""

    def __init__(self, mode="match"):
        """Retain test-only inference mode."""
        self.mode = mode

    def detect(self, frame, *, labels=None, confidence=0.5):
        """Return matching structured results or simulate a broken third-party detector."""
        if self.mode == "hang":
            time.sleep(30)
        if self.mode == "error":
            raise OSError("inference failed")
        if self.mode == "none":
            return []
        return [Detection("person", 0.9, (0, 0, frame.width, frame.height),
                          (0.5, 0.5), frame.sequence, frame.timestamp)]


@unittest.skipIf(cv2 is None, "optional numpy/opencv not installed")
class TestVision(unittest.TestCase):
    """Verify exact output contracts and negative cases using image fixtures."""

    def frame(self, rgb, sequence=1):
        """Encode one immutable RGB fixture and matching JPEG."""
        ok, jpeg = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        self.assertTrue(ok)
        return Frame(sequence, time.monotonic(), rgb.shape[1], rgb.shape[0],
                     rgb.tobytes(), jpeg.tobytes())

    def test_red_hue_wrap_boxes_and_immutable_annotation(self):
        image = np.zeros((40, 60, 3), dtype=np.uint8)
        image[10:30, 20:40] = (255, 0, 0)
        frame = self.frame(image)
        detections = detect_color(frame, hsv_ranges=hsv_target(0), min_area=100, label="red")
        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0].box, (20, 10, 20, 20))
        self.assertEqual(detections[0].center, (0.5, 0.5))
        self.assertEqual(detections[0].area, 361)
        self.assertIsNone(detections[0].confidence)
        self.assertEqual(frame.rgb, image.tobytes())
        jpeg = annotate(frame, detections)
        self.assertIsNotNone(cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR))
        with self.assertRaises(ValueError):
            annotate(self.frame(image, sequence=2), detections)
        self.assertEqual(detect_color(frame, hsv_ranges=hsv_target(80), min_area=10), [])
        self.assertEqual(len(hsv_target(175)), 2)
        for kwargs in (dict(hue=True), dict(hue=180), dict(hue=0, tolerance=100)):
            with self.assertRaises(ValueError):
                hsv_target(**kwargs)

    def test_floor_clear_blocked_and_dark_unknown(self):
        observer = FloorObserver((((0, 0, 80), (179, 40, 200)),))
        clear = np.full((40, 60, 3), 120, dtype=np.uint8)
        self.assertEqual(observer(self.frame(clear)).state, "clear")
        blocked = clear.copy()
        blocked[25:35, 25:35] = (255, 0, 0)
        self.assertEqual(observer(self.frame(blocked)).state, "blocked")
        self.assertEqual(observer(self.frame(np.zeros_like(clear))).state, "unknown")
        for ranges in ((), (((180, 0, 0), (180, 255, 255)),),
                       (((10, 20, 30), (5, 255, 255)),)):
            with self.assertRaises(ValueError):
                FloorObserver(ranges)

    def test_model_checksums_shape_class_filtering_and_preprocessing(self):
        frame = self.frame(np.full((40, 60, 3), 120, dtype=np.uint8))
        with tempfile.TemporaryDirectory() as directory:
            proto, weights = Path(directory) / "deploy.prototxt", Path(directory) / "weights"
            proto.write_bytes(b"test-proto")
            weights.write_bytes(b"test-weights")
            args = dict(prototxt_sha256=hashlib.sha256(proto.read_bytes()).hexdigest(),
                        weights_sha256=hashlib.sha256(weights.read_bytes()).hexdigest())
            with self.assertRaises(ValueError):
                MobileNetSSD(proto, weights, **dict(args, weights_sha256="0" * 64))
            net = Mock()
            net.forward.return_value = np.array([[[[0, 15, 0.9, 0.1, 0.2, 0.8, 0.9],
                                                  [0, 8, 0.4, 0, 0, 1, 1]]]],
                                                dtype=np.float32)
            with patch("cv2.dnn.readNetFromCaffe", return_value=net):
                detector = MobileNetSSD(proto, weights, **args)
            detections = detect_objects(frame, detector=detector, labels=["person"])
            self.assertEqual(len(detections), 1)
            self.assertEqual(detections[0].label, "person")
            self.assertEqual(detections[0].sequence, frame.sequence)
            self.assertEqual(net.setInput.call_args.args[0].shape, (1, 3, 300, 300))
            self.assertEqual(detect_objects(frame, detector=detector, labels=["cat"]), [])
            with self.assertRaises(ValueError):
                detect_objects(frame, detector=detector, labels=["custom-item"])
            net.forward.return_value = np.zeros((1, 2))
            with self.assertRaises(ValueError):
                detector.detect(frame)
            net.forward.return_value = np.array([[[[0, 25, 0.9, 0, 0, 1, 1]]]])
            with self.assertRaises(ValueError):
                detector.detect(frame)

    def test_find_object_match_no_match_and_model_errors(self):
        frame = self.frame(np.full((40, 60, 3), 120, dtype=np.uint8))
        camera = Mock()
        camera.capture.return_value = frame
        result = find_object(camera, detector=SearchDetector(), labels=["person"])
        self.assertEqual(result[0].label, "person")
        self.assertEqual(find_object(camera, detector=SearchDetector("none"), labels=["person"],
                                     timeout=2, max_frames=1), [])
        with self.assertRaisesRegex(RuntimeError, "inference failed"):
            find_object(camera, detector=SearchDetector("error"), labels=["person"])
        started = time.monotonic()
        with self.assertRaises(TimeoutError):
            find_object(camera, detector=SearchDetector("hang"), labels=["person"], timeout=0.2)
        self.assertLess(time.monotonic() - started, 1.2)

    @unittest.skipUnless(os.environ.get("RASPTANK_TEST_MODEL_DIR"),
                         "explicit verified local model assets not configured")
    def test_real_model_load_inference_and_spawned_search(self):
        """Exercise actual Caffe loading/forward/pickling on a synthetic no-object scene."""
        directory = Path(os.environ["RASPTANK_TEST_MODEL_DIR"])
        detector = MobileNetSSD(directory / "deploy.prototxt",
                                directory / "mobilenet_iter_73000.caffemodel",
                                prototxt_sha256=MOBILENET_PROTOTXT_SHA256,
                                weights_sha256=MOBILENET_WEIGHTS_SHA256)
        frame = self.frame(np.full((240, 320, 3), 120, dtype=np.uint8))
        self.assertEqual(detect_objects(frame, detector=detector, labels=["person"],
                                        confidence=0.99), [])
        camera = Mock()
        camera.capture.return_value = frame
        self.assertEqual(find_object(camera, detector=detector, labels=["person"],
                                     confidence=0.99, timeout=10, max_frames=1), [])


if __name__ == "__main__":
    unittest.main()
