import json
import unittest
from unittest import mock

import server


def fake_response(payload: dict) -> mock.MagicMock:
    resp = mock.MagicMock()
    resp.read.return_value = json.dumps(payload).encode()
    resp.__enter__.return_value = resp
    resp.__exit__.return_value = False
    return resp


class SentryDetectTests(unittest.TestCase):
    def make_store(self):
        return server.TelemetryStore(domain=0, robot_host="127.0.0.1")

    def test_no_head_frame(self):
        result = self.make_store().sentry_detect("head")
        self.assertFalse(result["ok"])
        self.assertIn("No head frame", result["error"])

    def test_unknown_feed(self):
        result = self.make_store().sentry_detect("thermal")
        self.assertFalse(result["ok"])
        self.assertIn("Unknown feed", result["error"])

    def test_upstream_url_carries_feed_for_per_feed_tracking(self):
        store = self.make_store()
        store.set_webcam_frame(b"jpeg-bytes")
        seen = {}

        def capture(req, timeout=None):
            seen["url"] = req.full_url
            return fake_response({"persons": []})

        with mock.patch.object(server.urllib.request, "urlopen", side_effect=capture):
            result = store.sentry_detect("webcam")
        self.assertTrue(result["ok"])
        self.assertIn("feed=webcam", seen["url"])

    def test_head_persons_passed_through(self):
        store = self.make_store()
        store.set_camera_frame(b"jpeg-bytes")
        persons = [{"x1": 0.1, "y1": 0.2, "x2": 0.3, "y2": 0.9,
                    "cx": 0.2, "cy": 0.55, "conf": 0.9}]
        with mock.patch.object(server.urllib.request, "urlopen",
                               return_value=fake_response({"persons": persons})):
            result = store.sentry_detect("head")
        self.assertTrue(result["ok"])
        self.assertEqual(result["feed"], "head")
        self.assertEqual(result["persons"], persons)
        self.assertIn("ts", result)

    def test_webcam_feed_uses_webcam_frame(self):
        store = self.make_store()
        store.set_webcam_frame(b"jpeg-bytes")
        with mock.patch.object(server.urllib.request, "urlopen",
                               return_value=fake_response({"persons": []})):
            result = store.sentry_detect("webcam")
        self.assertTrue(result["ok"])
        self.assertEqual(result["feed"], "webcam")
        self.assertEqual(result["persons"], [])

    def test_detect_service_down(self):
        store = self.make_store()
        store.set_camera_frame(b"jpeg-bytes")
        with mock.patch.object(server.urllib.request, "urlopen",
                               side_effect=OSError("boom")):
            result = store.sentry_detect("head")
        self.assertFalse(result["ok"])
        self.assertIn("unreachable", result["error"])


class ShrinkJpegTests(unittest.TestCase):
    def test_returns_original_without_cv2(self):
        with mock.patch.object(server, "cv2", None):
            self.assertEqual(server.shrink_jpeg_for_detection(b"frame"), b"frame")

    def test_returns_original_on_undecodable_input(self):
        if server.cv2 is None:
            self.skipTest("OpenCV not available")
        self.assertEqual(server.shrink_jpeg_for_detection(b"not-a-jpeg"), b"not-a-jpeg")

    def test_downscales_wide_jpeg(self):
        if server.cv2 is None or server.np is None:
            self.skipTest("OpenCV not available")
        big = server.np.full((720, 1280, 3), 128, dtype=server.np.uint8)
        ok, encoded = server.cv2.imencode(".jpg", big)
        self.assertTrue(ok)
        shrunk = server.shrink_jpeg_for_detection(encoded.tobytes(), max_width=640)
        img = server.cv2.imdecode(
            server.np.frombuffer(shrunk, server.np.uint8), server.cv2.IMREAD_COLOR)
        self.assertEqual(img.shape[1], 640)
        self.assertEqual(img.shape[0], 360)

    def test_keeps_small_jpeg_untouched(self):
        if server.cv2 is None or server.np is None:
            self.skipTest("OpenCV not available")
        small = server.np.full((240, 320, 3), 128, dtype=server.np.uint8)
        ok, encoded = server.cv2.imencode(".jpg", small)
        self.assertTrue(ok)
        self.assertEqual(server.shrink_jpeg_for_detection(encoded.tobytes()), encoded.tobytes())


class SentryStreamTests(unittest.TestCase):
    def make_store(self):
        return server.TelemetryStore(domain=0, robot_host="127.0.0.1")

    def test_worker_runs_only_while_subscribed(self):
        import time as _time
        store = self.make_store()
        store.sentry_stream_subscribe()
        _time.sleep(0.3)
        result, seq = store.wait_sentry_result(-1, timeout=1.0)
        self.assertIsNotNone(result)          # worker produced results
        self.assertGreater(seq, 0)
        thread = store.sentry_stream_thread
        store.sentry_stream_unsubscribe()
        thread.join(timeout=2.0)
        self.assertFalse(thread.is_alive())   # last client gone -> worker exits

    def test_wait_times_out_without_new_result(self):
        store = self.make_store()
        result, seq = store.wait_sentry_result(0, timeout=0.05)
        self.assertIsNone(result)
        self.assertEqual(seq, 0)


class SentryRouteTests(unittest.TestCase):
    def test_route_dispatched(self):
        with open("server.py") as fh:
            src = fh.read()
        self.assertIn('"/api/sentry/detect"', src)
        self.assertIn('"/api/sentry/stream"', src)


class SentryFrontendContractTests(unittest.TestCase):
    def test_ui_wiring_present(self):
        with open("static/index.html") as fh:
            html = fh.read()
        for needle in ("sentryToggle", "floatCamOverlay",
                       "floatWebcamOverlay", "floatCamSentry"):
            self.assertIn(needle, html)
        with open("static/app.js") as fh:
            js = fh.read()
        self.assertIn("/api/sentry/stream", js)
        self.assertIn("h1_sentry_mode", js)

    def test_lock_button_wiring_present(self):
        with open("static/index.html") as fh:
            html = fh.read()
        self.assertIn("floatWebcamTargets", html)
        with open("static/app.js") as fh:
            js = fh.read()
        for needle in ("target-lock-btn", "floatWebcamTargets", "LOCKED",
                       "sentryBoxesToggle", "h1_sentry_boxes"):
            self.assertIn(needle, js)
        self.assertIn("sentryBoxesToggle", html)
        with open("static/styles.css") as fh:
            css = fh.read()
        self.assertIn("target-pulse", css)


if __name__ == "__main__":
    unittest.main()
