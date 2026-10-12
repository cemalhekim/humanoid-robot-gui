import json
import unittest
from unittest import mock

import server


class _FakeResponse:
    def __init__(self, payload: dict) -> None:
        self._body = json.dumps(payload).encode()

    def read(self) -> bytes:
        return self._body

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


WLED_STATE = {"on": True, "bri": 128, "seg": [{"id": 0, "col": [[255, 160, 0], [0, 0, 0], [0, 0, 0]], "fx": 0}]}


class LightsTests(unittest.TestCase):
    def make_store(self):
        return server.TelemetryStore(domain=0, robot_host="127.0.0.1")

    def test_status_reads_wled_state(self):
        store = self.make_store()
        with mock.patch.object(server, "WLED_BASE_URL", "http://wled.test"), mock.patch.object(
            server.urllib.request, "urlopen", return_value=_FakeResponse(WLED_STATE)
        ) as urlopen:
            result = store.lights_status()
        self.assertTrue(result["enabled"])
        self.assertTrue(result["reachable"])
        self.assertEqual(result["hex"], "#ffa000")
        self.assertEqual(result["rgb"], [255, 160, 0])
        self.assertEqual(result["bri"], 128)
        self.assertTrue(result["on"])
        request = urlopen.call_args[0][0]
        self.assertEqual(request.full_url, "http://wled.test/json/state")
        self.assertEqual(request.get_method(), "GET")

    def test_status_unreachable_is_not_an_error(self):
        store = self.make_store()
        with mock.patch.object(server, "WLED_BASE_URL", "http://wled.test"), mock.patch.object(
            server.urllib.request, "urlopen", side_effect=OSError("down")
        ):
            result = store.lights_status()
        self.assertTrue(result["enabled"])
        self.assertFalse(result["reachable"])

    def test_status_disabled_without_url(self):
        store = self.make_store()
        with mock.patch.object(server, "WLED_BASE_URL", ""):
            self.assertFalse(store.lights_status()["enabled"])
            status, result = store.set_lights({"color": "#ff0000"})
        self.assertEqual(status, 503)
        self.assertFalse(result["ok"])

    def test_set_color_posts_solid_segment(self):
        store = self.make_store()
        reply = {**WLED_STATE, "seg": [{"id": 0, "col": [[255, 0, 0], [0, 0, 0], [0, 0, 0]], "fx": 0}]}
        with mock.patch.object(server, "WLED_BASE_URL", "http://wled.test"), mock.patch.object(
            server.urllib.request, "urlopen", return_value=_FakeResponse(reply)
        ) as urlopen:
            status, result = store.set_lights({"color": "#FF0000", "bri": 200})
        self.assertEqual(status, 200)
        self.assertTrue(result["ok"])
        self.assertEqual(result["hex"], "#ff0000")
        request = urlopen.call_args[0][0]
        self.assertEqual(request.get_method(), "POST")
        body = json.loads(request.data.decode())
        self.assertEqual(body["seg"], [{"id": 0, "col": [[255, 0, 0], [0, 0, 0], [0, 0, 0]], "fx": 0}])
        self.assertTrue(body["on"])
        self.assertEqual(body["bri"], 200)

    def test_set_rgb_and_off(self):
        store = self.make_store()
        with mock.patch.object(server, "WLED_BASE_URL", "http://wled.test"), mock.patch.object(
            server.urllib.request, "urlopen", return_value=_FakeResponse(WLED_STATE)
        ) as urlopen:
            status, _ = store.set_lights({"rgb": [1, 2, 3]})
            self.assertEqual(status, 200)
            self.assertEqual(json.loads(urlopen.call_args[0][0].data.decode())["seg"][0]["col"][0], [1, 2, 3])
            status, _ = store.set_lights({"on": False})
            self.assertEqual(status, 200)
            self.assertEqual(json.loads(urlopen.call_args[0][0].data.decode()), {"on": False, "v": True})

    def test_rejects_bad_input(self):
        store = self.make_store()
        with mock.patch.object(server, "WLED_BASE_URL", "http://wled.test"), mock.patch.object(
            server.urllib.request, "urlopen"
        ) as urlopen:
            for payload in ({"color": "red"}, {"color": "#12345"}, {"rgb": [0, 0]}, {"rgb": [0, 0, 300]},
                            {"bri": 256}, {"bri": "x"}, {"on": "yes"}, {}, None):
                status, result = store.set_lights(payload)
                self.assertEqual(status, 400, payload)
                self.assertFalse(result["ok"])
        urlopen.assert_not_called()

    def test_wled_down_is_502(self):
        store = self.make_store()
        with mock.patch.object(server, "WLED_BASE_URL", "http://wled.test"), mock.patch.object(
            server.urllib.request, "urlopen", side_effect=OSError("down")
        ):
            status, result = store.set_lights({"color": "#00ff00"})
        self.assertEqual(status, 502)
        self.assertFalse(result["ok"])

    def test_http_routes_are_registered(self):
        with open("server.py") as fh:
            source = fh.read()
        self.assertIn('request_path == "/api/lights"', source)
        self.assertGreaterEqual(source.count('"/api/lights",'), 2)

    def test_dashboard_has_color_wheel(self):
        with open("static/index.html") as fh:
            html = fh.read()
        self.assertIn('id="lightsColor"', html)
        self.assertIn('type="color"', html)
        with open("static/app.js") as fh:
            js = fh.read()
        self.assertIn('fetch("/api/lights"', js)


if __name__ == "__main__":
    unittest.main()
