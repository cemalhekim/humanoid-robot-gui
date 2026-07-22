import io
import json
import unittest
from unittest import mock

import server


class _Conn:
    """Socketless stand-in so BaseHTTPRequestHandler can parse a canned request."""

    def __init__(self, raw: bytes) -> None:
        self._raw = raw
        self.wfile = io.BytesIO()

    def makefile(self, mode: str, *args, **kwargs):
        return io.BytesIO(self._raw) if "r" in mode else self.wfile

    def sendall(self, data: bytes) -> None:
        self.wfile.write(data)


def post(store: server.TelemetryStore, path: str, payload: dict) -> tuple[int, dict]:
    """Run one POST through the real TelemetryHandler request parsing/routing."""
    body = json.dumps(payload).encode()
    raw = (
        f"POST {path} HTTP/1.1\r\nHost: t\r\nContent-Type: application/json\r\n"
        f"Content-Length: {len(body)}\r\nConnection: close\r\n\r\n"
    ).encode() + body
    conn = _Conn(raw)
    with mock.patch.object(server.TelemetryHandler, "store", store, create=True):
        server.TelemetryHandler(conn, ("127.0.0.1", 0), None)
    head, _, rest = conn.wfile.getvalue().partition(b"\r\n\r\n")
    status = int(head.split(b" ", 2)[1])
    try:
        decoded = json.loads(rest.split(b"\r\n\r\n")[-1] or b"{}")
    except ValueError:
        decoded = {}
    return status, decoded


class TrackingGatingTests(unittest.TestCase):
    def make_store(self):
        return server.TelemetryStore(domain=0, robot_host="127.0.0.1")

    def test_start_requires_risk_ack(self):
        store = self.make_store()
        status, response = store.request_track_start({})
        self.assertEqual(status, 403)
        self.assertFalse(response["ok"])

    def test_start_refused_when_feature_disabled(self):
        store = self.make_store()
        old = server.TRACKING_ENABLED
        server.TRACKING_ENABLED = False
        try:
            status, response = store.request_track_start(
                {"armed": True, "i_understand_risk": True}
            )
        finally:
            server.TRACKING_ENABLED = old
        self.assertEqual(status, 409)

    def test_start_without_dds_returns_503(self):
        store = self.make_store()
        old = server.TRACKING_ENABLED
        server.TRACKING_ENABLED = True
        try:
            status, response = store.request_track_start(
                {"armed": True, "i_understand_risk": True}
            )
        finally:
            server.TRACKING_ENABLED = old
        self.assertEqual(status, 503)  # no wrist_publisher offline

    def test_stop_is_idempotent(self):
        store = self.make_store()
        status, response = store.request_track_stop()
        self.assertEqual(status, 200)
        self.assertTrue(response["ok"])

    def test_snapshot_shape(self):
        store = self.make_store()
        snap = store.track_snapshot()
        for key in ("enabled", "active", "phase", "message", "updated_at"):
            self.assertIn(key, snap)


class TrackingRouteTests(unittest.TestCase):
    def test_routes_are_dispatched(self):
        with open("server.py") as fh:
            src = fh.read()
        self.assertIn('"/api/track/start"', src)
        self.assertIn('"/api/track/stop"', src)
        self.assertIn('"/api/track/status"', src)

    def test_post_track_start_reaches_handler_with_payload(self):
        # Regression: the paths had handlers but were missing from do_POST's
        # route allowlist (404 before dispatch) and the JSON-parse list (empty
        # payload -> 403 even with acks). Feature-disabled must yield 409,
        # proving both routing and body parsing ran.
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        with mock.patch.object(server, "TRACKING_ENABLED", False):
            status, response = post(
                store, "/api/track/start", {"armed": True, "i_understand_risk": True}
            )
        self.assertEqual(status, 409)
        self.assertIn("disabled", response.get("error", ""))

    def test_post_track_stop_is_dispatched(self):
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        status, response = post(store, "/api/track/stop", {})
        self.assertEqual(status, 200)
        self.assertTrue(response["ok"])


class TrackToolTests(unittest.TestCase):
    def make_store(self):
        return server.TelemetryStore(domain=0, robot_host="127.0.0.1")

    def test_tool_hidden_when_disabled(self):
        old = server.LLM_TOOL_TRACK_ENABLED
        server.LLM_TOOL_TRACK_ENABLED = False
        try:
            specs = self.make_store().chat_tool_specs()
        finally:
            server.LLM_TOOL_TRACK_ENABLED = old
        names = [s["function"]["name"] for s in specs]
        self.assertNotIn("track_person", names)

    def test_tool_requires_confirm(self):
        old = server.LLM_TOOL_TRACK_ENABLED
        server.LLM_TOOL_TRACK_ENABLED = True
        try:
            result = self.make_store().run_chat_tool("track_person", {"action": "start"})
        finally:
            server.LLM_TOOL_TRACK_ENABLED = old
        self.assertFalse(result["ok"])
        self.assertIn("confirm", result["error"])

    def test_tool_stop_works_offline(self):
        old = server.LLM_TOOL_TRACK_ENABLED
        server.LLM_TOOL_TRACK_ENABLED = True
        try:
            result = self.make_store().run_chat_tool(
                "track_person", {"action": "stop", "confirm": True}
            )
        finally:
            server.LLM_TOOL_TRACK_ENABLED = old
        self.assertTrue(result["ok"])


if __name__ == "__main__":
    unittest.main()
