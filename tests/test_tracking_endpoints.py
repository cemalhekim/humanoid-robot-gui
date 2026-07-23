import io
import json
import threading
import unittest
from tempfile import TemporaryDirectory
from types import SimpleNamespace
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

    def test_sentry_lock_requires_webcam_target(self):
        store = self.make_store()
        store.sentry_mode_on = True
        with mock.patch.object(server, "TRACKING_ENABLED", True):
            status, response = store.request_track_start({
                "armed": True,
                "i_understand_risk": True,
                "source": "sentry-lock",
                "camera": "head",
            })
        self.assertEqual(status, 400)
        self.assertIn("webcam target", response["error"])

    def test_locked_webcam_config_reaches_session_snapshot(self):
        store = self.make_store()
        store.sentry_mode_on = True
        store.wrist_publisher = object()
        store.lowstate_msg = object()
        store.lowcmd_factory = object()
        store.crc = object()
        payload = {
            "armed": True,
            "i_understand_risk": True,
            "source": "sentry-lock",
            "camera": "webcam",
            "permanent": True,
            "closed_loop": True,
            "target": {"cx": 0.4, "cy": 0.3},
            "target_id": 21,
        }
        with mock.patch.object(server, "TRACKING_ENABLED", True):
            with mock.patch.object(store, "_suspend_xr_motion_publishers",
                                   return_value={"ok": True}):
                with mock.patch.object(threading.Thread, "start"):
                    status, response = store.request_track_start(payload)
        self.assertEqual(status, 200)
        tracking_status = response["tracking"]
        self.assertEqual(tracking_status["camera"], "webcam")
        self.assertEqual(tracking_status["target_id"], 21)
        self.assertTrue(tracking_status["permanent"])
        self.assertTrue(tracking_status["closed_loop"])


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


class TrackPayloadTests(unittest.TestCase):
    def test_deployed_webcam_uses_robot_relative_horizontal_orientation(self):
        self.assertGreater(server.SENTRY_FOV_YAW, 0.0)

    def test_sentry_response_is_faster_but_still_bounded(self):
        self.assertGreater(
            server.SENTRY_REPLAY_RESPONSE,
            server.ARM_REPLAY_RESPONSE_DEFAULT,
        )
        self.assertLessEqual(
            server.SENTRY_REPLAY_RESPONSE,
            server.ARM_REPLAY_RESPONSE_LEGACY_MAX,
        )
        self.assertGreater(server.SENTRY_MAX_STEP_RAD_S, 0.45)
        self.assertLessEqual(server.SENTRY_MAX_STEP_RAD_S, 1.0)

    def test_parses_locked_webcam_session(self):
        parsed = server.parse_track_payload({
            "camera": "webcam",
            "permanent": True,
            "closed_loop": True,
            "target": {"cx": 0.25, "cy": 0.75},
            "target_id": 17,
        })
        self.assertEqual(parsed["camera"], "webcam")
        self.assertEqual(parsed["target_id"], 17)
        self.assertEqual(parsed["target"], {"cx": 0.25, "cy": 0.75})
        self.assertTrue(parsed["permanent"])
        self.assertTrue(parsed["closed_loop"])

    def test_rejects_invalid_camera_and_target(self):
        with self.assertRaises(ValueError):
            server.parse_track_payload({"camera": "thermal"})
        with self.assertRaises(ValueError):
            server.parse_track_payload({"target": {"cx": 1.5, "cy": 0.5}})

    def test_rejects_non_boolean_control_flags(self):
        with self.assertRaises(ValueError):
            server.parse_track_payload({"permanent": "yes"})
        with self.assertRaises(ValueError):
            server.parse_track_payload({"closed_loop": 1})


class SentryConstantHeightTests(unittest.TestCase):
    def test_horizontal_targets_share_center_hand_z(self):
        mapper = server.tracking.PointingMapper(
            fov_yaw_rad=server.SENTRY_FOV_YAW,
            fov_pitch_rad=server.SENTRY_FOV_PITCH,
            yaw_offset=server.SENTRY_YAW_OFFSET,
            pitch_offset=server.SENTRY_PITCH_OFFSET,
            dead_band=0.0,
        )
        target_z = server.sentry_right_hand_z(mapper.targets(0.5, 0.5))
        self.assertIsNotNone(target_z)

        for cx, cy in ((0.0, 0.1), (0.2, 0.8), (0.5, 0.5), (0.8, 0.2), (1.0, 0.9)):
            goal = server.sentry_constant_hand_z_goal(mapper.targets(cx, cy), target_z)
            self.assertAlmostEqual(
                server.sentry_right_hand_z(goal),
                target_z,
                delta=1e-5,
            )
            pitch = goal[server.tracking.R_SHOULDER_PITCH]
            lo, hi = server.tracking.TRACK_LIMITS[server.tracking.R_SHOULDER_PITCH]
            self.assertGreaterEqual(pitch, lo)
            self.assertLessEqual(pitch, hi)

    def test_rate_limited_horizontal_sweep_remains_level(self):
        mapper = server.tracking.PointingMapper(dead_band=0.0)
        target_z = server.sentry_right_hand_z(mapper.targets(0.5, 0.5))
        previous = server.sentry_constant_hand_z_goal(
            mapper.targets(0.5, 0.5), target_z
        )
        limiter = server.tracking.RateLimiter(max_step_rad_s=0.65)
        dt = 0.125

        for cx in (0.7, 0.9, 1.0, 0.6, 0.2, 0.0):
            goal = server.sentry_constant_hand_z_goal(
                mapper.targets(cx, 0.1), target_z
            )
            stepped = limiter.step(previous, goal, dt)
            current = server.sentry_constant_hand_z_step(
                previous, stepped, target_z, limiter.max_step_rad_s * dt
            )
            self.assertLessEqual(
                abs(
                    current[server.tracking.R_SHOULDER_PITCH]
                    - previous[server.tracking.R_SHOULDER_PITCH]
                ),
                limiter.max_step_rad_s * dt + 1e-12,
            )
            self.assertAlmostEqual(
                server.sentry_right_hand_z(current),
                target_z,
                delta=1e-5,
            )
            previous = current

    def test_height_correction_cannot_jump_past_pitch_rate_limit(self):
        previous = dict(server.tracking.NEUTRAL_TEMPLATE)
        stepped = dict(previous)
        target_z = server.sentry_right_hand_z(server.tracking.POINTING_TEMPLATE)
        max_step = 0.05
        corrected = server.sentry_constant_hand_z_step(
            previous, stepped, target_z, max_step
        )
        self.assertLessEqual(
            abs(
                corrected[server.tracking.R_SHOULDER_PITCH]
                - previous[server.tracking.R_SHOULDER_PITCH]
            ),
            max_step,
        )


class SentryArmingTests(unittest.TestCase):
    """Sentry ON enables prediction; an explicit person lock starts motion."""

    def make_store(self):
        return server.TelemetryStore(domain=0, robot_host="127.0.0.1")

    def test_sentry_on_is_motion_free(self):
        store = self.make_store()
        with mock.patch.object(store, "request_track_start") as start:
            status, response = store.set_sentry_mode({"on": True})
        self.assertEqual(status, 200)
        self.assertTrue(response["sentry_mode"])
        self.assertFalse(response["tracking"]["active"])
        start.assert_not_called()

    def test_sentry_on_does_not_cancel_unrelated_replay(self):
        store = self.make_store()
        ev = threading.Event()
        thread = threading.Thread(target=ev.wait, daemon=True)
        thread.start()
        store.replay_thread = thread
        store.replay_cancel = ev
        status, response = store.set_sentry_mode({"on": True})
        self.assertEqual(status, 200)
        self.assertFalse(ev.is_set())
        self.assertTrue(response["sentry_mode"])
        ev.set()

    def test_track_stop_keeps_sentry_armed_for_another_lock(self):
        store = self.make_store()
        store.set_sentry_mode({"on": True})
        store.request_track_stop()
        self.assertTrue(store.track_snapshot()["sentry_mode"])

    def test_session_natural_end_keeps_sentry_armed(self):
        store = self.make_store()
        store.sentry_mode_on = True
        with mock.patch.object(store, "request_home", return_value=(404, {"ok": False})):
            with mock.patch.object(server, "TRACKING_MAX_SESSION_S", -1.0):
                store._run_tracking(threading.Event())
        snap = store.track_snapshot()
        self.assertTrue(snap["sentry_mode"])
        self.assertFalse(snap["active"])

    def test_sentry_off_disarms_and_stops_tracking(self):
        store = self.make_store()
        store.sentry_mode_on = True
        with mock.patch.object(store, "request_track_stop", wraps=store.request_track_stop) as stop:
            status, response = store.set_sentry_mode({"on": False})
        self.assertEqual(status, 200)
        self.assertFalse(response["sentry_mode"])
        stop.assert_called_once()


class HomeMoveTests(unittest.TestCase):
    """The Home button and every physical tracking-session end send the arm
    to the saved home pose; Sentry prediction may remain armed."""

    def make_store(self):
        return server.TelemetryStore(domain=0, robot_host="127.0.0.1")

    def test_home_moves_to_saved_home_position(self):
        store = self.make_store()
        with TemporaryDirectory() as directory:
            (server.Path(directory) / "20260714-111543-home.pose.json").write_text(
                "{}", encoding="utf-8")
            with mock.patch.object(server, "RECORDINGS_DIR", server.Path(directory)):
                with mock.patch.object(
                    store, "request_robot_replay", return_value=(202, {"ok": True})
                ) as replay:
                    status, response = store.request_home()
        self.assertEqual(status, 202)
        self.assertTrue(response["ok"])
        payload = replay.call_args[0][0]
        self.assertEqual(payload["filename"], "20260714-111543-home.pose.json")
        self.assertTrue(payload["execute_arm_sdk"])
        self.assertEqual(payload["command_scope"], "arms")
        self.assertTrue(payload["closed_loop"])

    def test_home_errors_without_saved_home_position(self):
        store = self.make_store()
        with TemporaryDirectory() as directory:
            with mock.patch.object(server, "RECORDINGS_DIR", server.Path(directory)):
                status, response = store.request_home()
        self.assertEqual(status, 404)
        self.assertIn("home", response["error"])

    def test_session_natural_end_triggers_home_move(self):
        store = self.make_store()
        store.sentry_mode_on = True
        with mock.patch.object(
            store, "request_home", return_value=(404, {"ok": False})
        ) as home:
            with mock.patch.object(server, "TRACKING_MAX_SESSION_S", -1.0):
                store._run_tracking(threading.Event())
        home.assert_called_once()

    def test_operator_stop_triggers_home_move(self):
        store = self.make_store()
        cancel = threading.Event()
        cancel.set()
        with mock.patch.object(
            store, "request_home", return_value=(404, {"ok": False})
        ) as home:
            store._run_tracking(cancel)
        home.assert_called_once()


class TrackingLoopTests(unittest.TestCase):
    def test_webcam_lock_uses_shared_stream_closed_loop_and_real_publisher(self):
        store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")
        motors = [SimpleNamespace(q=0.0, dq=0.0, tau_est=0.0) for _ in range(35)]
        store.lowstate_msg = SimpleNamespace(motor_state=motors)
        cancel = threading.Event()
        publisher = mock.MagicMock()
        publisher.Write.side_effect = lambda _cmd: cancel.set()
        store.wrist_publisher = publisher
        store.track_config = {
            "camera": "webcam",
            "permanent": True,
            "closed_loop": True,
            "target": {"cx": 0.4, "cy": 0.3},
            "target_id": 17,
        }
        person = {
            "id": 17,
            "cx": 0.4,
            "cy": 0.5,
            "x1": 0.2,
            "x2": 0.6,
            "y1": 0.1,
            "y2": 0.9,
            "conf": 0.9,
            "head": {"x": 0.4, "y": 0.2},
        }
        with mock.patch.object(store, "sentry_stream_subscribe") as subscribe:
            with mock.patch.object(store, "sentry_stream_unsubscribe") as unsubscribe:
                with mock.patch.object(
                    store,
                    "wait_sentry_result",
                    return_value=({"ok": True, "persons": [person]}, 1),
                ):
                    with mock.patch.object(
                        store,
                        "_closed_loop_arm_targets",
                        side_effect=lambda _msg, desired, _state, _dt, _tuning: (
                            desired, {}, {}
                        ),
                    ) as closed_loop:
                        with mock.patch.object(
                            store, "_build_arm_sdk_trajectory_cmd", return_value=object()
                        ):
                            with mock.patch.object(
                                store, "request_home", return_value=(200, {"ok": True})
                            ):
                                store._run_tracking(cancel)
        subscribe.assert_called_once()
        unsubscribe.assert_called_once()
        closed_loop.assert_called()
        tuning = closed_loop.call_args.args[4]
        self.assertEqual(tuning["response"], server.SENTRY_REPLAY_RESPONSE)
        publisher.Write.assert_called_once()


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

    def test_detection_border_matches_lock_button_state(self):
        # Both the bounding-box border and confidence label follow the same
        # unlocked-green / locked-red convention as the lock button.
        with open("static/app.js") as fh:
            src = fh.read()
        render_boxes = src.split("const renderBoxes =", 1)[1].split(
            "const renderCounter =", 1
        )[0]
        color_rule = 'locked ? "#e60000" : "#12b252"'
        self.assertEqual(render_boxes.count(color_rule), 2)


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
