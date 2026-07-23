import io
import json
import threading
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
        store.set_sentry_mode({"on": True})
        old = server.TRACKING_ENABLED
        server.TRACKING_ENABLED = True
        try:
            status, response = store.request_track_start(
                {"armed": True, "i_understand_risk": True}
            )
        finally:
            server.TRACKING_ENABLED = old
        self.assertEqual(status, 503)  # no wrist_publisher offline

    def test_start_refused_while_sentry_mode_off(self):
        # Sentry Mode is the operator's master switch (2026-07-22): with it
        # off, tracking must never start — from any caller.
        store = self.make_store()
        old = server.TRACKING_ENABLED
        server.TRACKING_ENABLED = True
        try:
            status, response = store.request_track_start(
                {"armed": True, "i_understand_risk": True}
            )
        finally:
            server.TRACKING_ENABLED = old
        self.assertEqual(status, 409)
        self.assertIn("Sentry", response["error"])

    def test_sentry_mode_off_stops_session_and_shows_in_snapshot(self):
        store = self.make_store()
        store.set_sentry_mode({"on": True})
        self.assertTrue(store.track_snapshot()["sentry_mode"])
        status, response = store.set_sentry_mode({"on": False})
        self.assertEqual(status, 200)
        snap = store.track_snapshot()
        self.assertFalse(snap["sentry_mode"])
        self.assertFalse(snap["active"])
        self.assertEqual(snap["phase"], "idle")

    def test_sentry_mode_requires_boolean(self):
        store = self.make_store()
        status, response = store.set_sentry_mode({"on": "yes"})
        self.assertEqual(status, 400)

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

    def test_post_sentry_mode_is_dispatched_with_payload(self):
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        status, response = post(store, "/api/sentry/mode", {"on": True})
        self.assertEqual(status, 200)
        self.assertTrue(response["sentry_mode"])


class SentryFollowInvariantTests(unittest.TestCase):
    """Sentry Mode <=> tracking session (operator invariant, 2026-07-23):
    on must mean the session runs, off must mean it never does."""

    def make_store(self):
        return server.TelemetryStore(domain=0, robot_host="127.0.0.1")

    def test_sentry_on_auto_starts_tracking(self):
        # Turning Sentry on IS the start command. Offline (no DDS) the auto
        # start must reach the arm_sdk publisher gate (503) — proving it went
        # past the sentry/risk-ack gates without a separate start call.
        store = self.make_store()
        with mock.patch.object(server, "TRACKING_ENABLED", True):
            status, response = store.set_sentry_mode({"on": True})
        self.assertEqual(status, 200)
        self.assertTrue(response["sentry_mode"])
        self.assertEqual(response["start"]["status"], 503)
        self.assertIn("arm_sdk", response["start"]["error"])

    def test_sentry_on_cancels_blocking_replay(self):
        # A held arm replay must not block the master switch: sentry-on
        # cancels it, then the start attempt reaches the DDS gate.
        store = self.make_store()
        ev = threading.Event()
        thread = threading.Thread(target=ev.wait, daemon=True)
        thread.start()
        store.replay_thread = thread
        store.replay_cancel = ev
        with mock.patch.object(server, "TRACKING_ENABLED", True):
            status, response = store.set_sentry_mode({"on": True})
        self.assertEqual(status, 200)
        self.assertTrue(ev.is_set())
        self.assertEqual(response["start"]["status"], 503)

    def test_track_stop_turns_sentry_off(self):
        store = self.make_store()
        store.set_sentry_mode({"on": True})
        store.request_track_stop()
        self.assertFalse(store.track_snapshot()["sentry_mode"])

    def test_session_natural_end_turns_sentry_off(self):
        # Ceiling/abort ends must not leave sentry claiming a session runs.
        store = self.make_store()
        store.sentry_mode_on = True
        with mock.patch.object(server, "TRACKING_MAX_SESSION_S", -1.0):
            store._run_tracking(threading.Event())
        snap = store.track_snapshot()
        self.assertFalse(snap["sentry_mode"])
        self.assertFalse(snap["active"])


class SentryUiSourceTests(unittest.TestCase):
    def test_toggle_state_comes_from_server_not_localstorage(self):
        # The dashboard toggle must render the SERVER's sentry flag; the old
        # localStorage-as-truth push caused UI-on/server-off drift (2026-07-23).
        with open("static/app.js") as fh:
            src = fh.read()
        self.assertNotIn('"h1_sentry_mode"', src)
        self.assertNotIn("pushMode(isOn())", src)

    def test_lock_button_colors_swapped(self):
        # Operator request 2026-07-23: unlocked = green, locked = red.
        with open("static/styles.css") as fh:
            css = fh.read()
        unlocked = css.split(".target-lock-btn {", 1)[1][:600]
        locked = css.split(".target-lock-btn.locked", 1)[1][:300]
        self.assertIn("rgba(18, 178, 82", unlocked)
        self.assertIn("rgba(230, 0, 0", locked)


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
