from __future__ import annotations

import io
import json
import unittest
from unittest import mock

import server


def fake_response(payload: object) -> io.BytesIO:
    body = io.BytesIO(json.dumps(payload).encode("utf-8"))
    body.__enter__ = lambda *args: body  # type: ignore[attr-defined]
    body.__exit__ = lambda *args: False  # type: ignore[attr-defined]
    return body


class SmartplugTest(unittest.TestCase):
    def test_status_without_token_reports_not_configured(self) -> None:
        with mock.patch.object(server, "HA_TOKEN", ""):
            status, payload = server.smartplug_status()
        self.assertEqual(status, 200)
        self.assertTrue(payload["ok"])
        self.assertFalse(payload["enabled"])
        self.assertEqual(payload["state"], "unavailable")
        self.assertIn("HA_TOKEN", payload["error"])

    def test_toggle_without_token_is_rejected(self) -> None:
        with mock.patch.object(server, "HA_TOKEN", ""):
            status, payload = server.smartplug_toggle()
        self.assertEqual(status, 503)
        self.assertFalse(payload["ok"])

    def test_status_parses_home_assistant_state(self) -> None:
        ha_state = {
            "entity_id": server.HA_SWITCH_ENTITY,
            "state": "on",
            "attributes": {"friendly_name": "SomoffSwitch2408"},
        }
        with mock.patch.object(server, "HA_TOKEN", "token"), mock.patch(
            "urllib.request.urlopen", return_value=fake_response(ha_state)
        ) as urlopen:
            status, payload = server.smartplug_status()
        self.assertEqual(status, 200)
        self.assertTrue(payload["enabled"])
        self.assertEqual(payload["state"], "on")
        self.assertEqual(payload["friendly_name"], "SomoffSwitch2408")
        request = urlopen.call_args[0][0]
        self.assertEqual(request.get_method(), "GET")
        self.assertEqual(request.get_header("Authorization"), "Bearer token")
        self.assertTrue(request.full_url.endswith(f"/api/states/{server.HA_SWITCH_ENTITY}"))

    def test_toggle_returns_new_state_from_service_call(self) -> None:
        changed = [
            {"entity_id": server.HA_SWITCH_ENTITY, "state": "off", "attributes": {}},
            {"entity_id": "switch.other", "state": "on", "attributes": {}},
        ]
        with mock.patch.object(server, "HA_TOKEN", "token"), mock.patch(
            "urllib.request.urlopen", return_value=fake_response(changed)
        ) as urlopen:
            status, payload = server.smartplug_toggle()
        self.assertEqual(status, 200)
        self.assertEqual(payload["state"], "off")
        request = urlopen.call_args[0][0]
        self.assertEqual(request.get_method(), "POST")
        self.assertTrue(request.full_url.endswith("/api/services/switch/toggle"))
        self.assertEqual(json.loads(request.data.decode("utf-8")), {"entity_id": server.HA_SWITCH_ENTITY})

    def test_toggle_requeries_when_entity_missing_from_service_reply(self) -> None:
        ha_state = {"entity_id": server.HA_SWITCH_ENTITY, "state": "on", "attributes": {}}
        with mock.patch.object(server, "HA_TOKEN", "token"), mock.patch(
            "urllib.request.urlopen", side_effect=[fake_response([]), fake_response(ha_state)]
        ):
            status, payload = server.smartplug_toggle()
        self.assertEqual(status, 200)
        self.assertEqual(payload["state"], "on")


if __name__ == "__main__":
    unittest.main()
