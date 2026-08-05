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


class PowerStatusTests(unittest.TestCase):
    def make_store(self):
        return server.TelemetryStore(domain=0, robot_host="127.0.0.1")

    def test_without_token_reports_unknown_and_disabled(self):
        store = self.make_store()
        with mock.patch.object(server, "HA_TOKEN", ""):
            result = store.power_status()
        self.assertEqual(result, {"enabled": False, "state": "unknown"})

    def test_reads_state_from_home_assistant(self):
        store = self.make_store()
        with mock.patch.object(server, "HA_TOKEN", "secret"), mock.patch.object(
            server.urllib.request, "urlopen",
            return_value=_FakeResponse({"state": "on"}),
        ) as urlopen:
            result = store.power_status()
        self.assertEqual(result["state"], "on")
        self.assertTrue(result["enabled"])
        request = urlopen.call_args[0][0]
        self.assertIn("/api/states/", request.full_url)
        self.assertEqual(request.get_header("Authorization"), "Bearer secret")

    def test_home_assistant_failure_reports_unknown(self):
        store = self.make_store()
        with mock.patch.object(server, "HA_TOKEN", "secret"), mock.patch.object(
            server.urllib.request, "urlopen", side_effect=OSError("down"),
        ):
            result = store.power_status()
        self.assertEqual(result["state"], "unknown")
        self.assertTrue(result["enabled"])

    def test_state_is_cached_briefly(self):
        store = self.make_store()
        with mock.patch.object(server, "HA_TOKEN", "secret"), mock.patch.object(
            server.urllib.request, "urlopen",
            return_value=_FakeResponse({"state": "off"}),
        ) as urlopen:
            store.power_status()
            result = store.power_status()
        self.assertEqual(urlopen.call_count, 1)
        self.assertEqual(result["state"], "off")


if __name__ == "__main__":
    unittest.main()
