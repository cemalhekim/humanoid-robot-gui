from __future__ import annotations

import io
import json
import unittest
from unittest import mock

import server


SNAPSHOT = {
    "connected": True,
    "sample_rate_hz": 500.0,
    "motor_count": 35,
    "analysis": {
        "health": {"state": "warning", "flags": [{"level": "warn", "message": "hot wrist"}]},
        "imu": {"roll_deg": -1.6, "pitch_deg": 1.5, "yaw_deg": 0.5, "temperature": 80},
        "motors": {
            "real_count": 27,
            "moving_count": 0,
            "hottest": {"name": "RightWristRoll", "value": 48.0},
            "max_abs_tau": {"name": "LeftAnklePitch", "value": 15.7},
            "groups": {
                "left_arm": {"count": 7, "moving": 0, "max_temperature": 45.0},
                "reserved": {"count": 8},
            },
        },
    },
    "hands": {"connected": True, "joint_count": 12},
    "battery": {"state": "not exposed"},
    "network": {"host": {"type": "Wi-Fi", "host": "10.2.100.142", "quality": "Connected"}},
}


class TelemetryContextTest(unittest.TestCase):
    def test_context_is_compact_and_grounded(self) -> None:
        ctx = server.build_telemetry_context(SNAPSHOT)
        self.assertIn("health=warning", ctx)
        self.assertIn("hot wrist", ctx)
        self.assertIn("hottest=RightWristRoll@48.0C", ctx)
        self.assertIn("left_arm", ctx)
        # Reserved (padding) slots are intentionally omitted from the summary.
        self.assertNotIn("reserved:", ctx)

    def test_context_handles_empty_snapshot(self) -> None:
        self.assertEqual(
            server.build_telemetry_context({}),
            "connected=None sample_rate_hz=None motor_count=None samples=None",
        )
        self.assertEqual(server.build_telemetry_context(None), "No telemetry available.")

    def test_context_includes_full_lowstate_and_ros_graph(self) -> None:
        snapshot = dict(SNAPSHOT)
        snapshot["motors"] = [
            {"index": 0, "name": "LeftHipYaw", "mode": 1, "q": 0.0012, "dq": 0.0,
             "tau_est": -0.12, "temperature": 45.0, "vol": 48.2},
            {"index": 34, "name": None, "mode": 0},  # reserved slot, skipped
        ]
        snapshot["hands"] = {"topic": "rt/inspire/state", "connected": True, "joint_count": 1,
                             "joints": [{"index": 0, "name": "RightPinky", "q": 0.5, "dq": 0.0,
                                         "tau_est": 0.0, "temperature": 30.0}]}
        snapshot["imu"] = {"quaternion": [1, 0, 0, 0], "gyroscope": [0, 0, 0],
                          "accelerometer": [0, 0, 9.8], "rpy": [0, 0, 0], "temperature": 80}
        snapshot["robot"] = {"mode_pr": 0, "mode_machine": 6, "tick": 123, "crc": 9}
        ros_graph = {"interface": "eth0", "nodes": ["/telemetry"],
                     "topics": {"/rt/lowstate": ["unitree_hg/msg/LowState"]}}
        ctx = server.build_telemetry_context(snapshot, ros_graph)
        self.assertIn("BODY MOTORS", ctx)
        self.assertIn("0 LeftHipYaw mode=1 q=0.001", ctx)
        self.assertNotIn("34 None", ctx)  # reserved slot excluded
        self.assertIn("HANDS", ctx)
        self.assertIn("RightPinky", ctx)
        self.assertIn("IMU raw", ctx)
        self.assertIn("mode_machine=6", ctx)
        self.assertIn("ROS 2 GRAPH", ctx)
        self.assertIn("/rt/lowstate [unitree_hg/msg/LowState]", ctx)


class ChatValidationTest(unittest.TestCase):
    def setUp(self) -> None:
        self.store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")

    def test_rejects_empty_messages(self) -> None:
        status, response = self.store.chat({"messages": []})
        self.assertEqual(status, 400)
        self.assertFalse(response["ok"])

    def test_rejects_bad_role(self) -> None:
        status, response = self.store.chat({"messages": [{"role": "system", "content": "hi"}]})
        self.assertEqual(status, 400)
        self.assertIn("role", response["error"])

    def test_rejects_when_last_message_not_user(self) -> None:
        status, response = self.store.chat(
            {"messages": [{"role": "user", "content": "hi"}, {"role": "assistant", "content": "yo"}]}
        )
        self.assertEqual(status, 400)
        self.assertIn("last message", response["error"])

    def test_rejects_too_many_messages(self) -> None:
        messages = [{"role": "user", "content": "x"}] * (server.LLM_MAX_MESSAGES + 1)
        status, response = self.store.chat({"messages": messages})
        self.assertEqual(status, 400)
        self.assertIn("Too many", response["error"])

    def test_disabled_returns_503(self) -> None:
        with mock.patch.object(server, "LLM_ENABLED", False):
            status, response = self.store.chat({"messages": [{"role": "user", "content": "hi"}]})
        self.assertEqual(status, 503)

    def test_success_injects_context_and_forwards(self) -> None:
        captured: dict = {}

        def fake_call_llm(messages):
            captured["messages"] = messages
            return 200, {"ok": True, "reply": "The hottest joint is RightWristRoll at 48C."}

        with mock.patch.object(self.store, "snapshot", return_value=SNAPSHOT), mock.patch.object(
            server, "call_llm", side_effect=fake_call_llm
        ):
            status, response = self.store.chat({"messages": [{"role": "user", "content": "hottest joint?"}]})

        self.assertEqual(status, 200)
        self.assertTrue(response["ok"])
        # System prompt must be first and contain live telemetry context.
        self.assertEqual(captured["messages"][0]["role"], "system")
        self.assertIn("TELEMETRY SNAPSHOT", captured["messages"][0]["content"])
        self.assertIn("RightWristRoll", captured["messages"][0]["content"])
        self.assertEqual(captured["messages"][-1]["content"], "hottest joint?")

    def test_content_is_truncated(self) -> None:
        long = "a" * (server.LLM_MAX_MESSAGE_CHARS + 500)
        captured: dict = {}

        def fake_call_llm(messages):
            captured["messages"] = messages
            return 200, {"ok": True, "reply": "ok"}

        with mock.patch.object(self.store, "snapshot", return_value=SNAPSHOT), mock.patch.object(
            server, "call_llm", side_effect=fake_call_llm
        ):
            self.store.chat({"messages": [{"role": "user", "content": long}]})

        self.assertEqual(len(captured["messages"][-1]["content"]), server.LLM_MAX_MESSAGE_CHARS)


class CallLlmTest(unittest.TestCase):
    def test_parses_openai_response(self) -> None:
        body = json.dumps(
            {"model": "qwen3", "choices": [{"message": {"content": "hello"}}], "usage": {"total_tokens": 5}}
        ).encode("utf-8")
        with mock.patch("urllib.request.urlopen", return_value=_fake_response(body)):
            status, response = server.call_llm([{"role": "user", "content": "hi"}])
        self.assertEqual(status, 200)
        self.assertEqual(response["reply"], "hello")
        self.assertEqual(response["usage"], {"total_tokens": 5})

    def test_maps_unreachable_endpoint(self) -> None:
        import urllib.error

        with mock.patch("urllib.request.urlopen", side_effect=urllib.error.URLError("refused")):
            status, response = server.call_llm([{"role": "user", "content": "hi"}])
        self.assertEqual(status, 503)
        self.assertFalse(response["ok"])
        self.assertIn("Cannot reach LLM", response["error"])

    def test_maps_malformed_response(self) -> None:
        with mock.patch("urllib.request.urlopen", return_value=_fake_response(b'{"choices": []}')):
            status, response = server.call_llm([{"role": "user", "content": "hi"}])
        self.assertEqual(status, 502)
        self.assertFalse(response["ok"])


class VoiceTest(unittest.TestCase):
    def test_stt_disabled_returns_503(self) -> None:
        with mock.patch.object(server, "LLM_STT_ENABLED", False):
            status, response = server.transcribe_audio(b"audio", "audio/webm")
        self.assertEqual(status, 503)
        self.assertFalse(response["ok"])

    def test_tts_disabled_returns_503(self) -> None:
        with mock.patch.object(server, "LLM_TTS_ENABLED", False):
            status, result, ctype = server.synthesize_speech("hi")
        self.assertEqual(status, 503)
        self.assertEqual(ctype, "")

    def test_multipart_body_is_well_formed(self) -> None:
        boundary, body = server._multipart_audio(
            b"RAWBYTES", "speech.webm", "audio/webm", {"model": "m", "response_format": "json"}
        )
        self.assertIn(boundary, body.decode("latin1"))
        self.assertIn(b'name="model"', body)
        self.assertIn(b'filename="speech.webm"', body)
        self.assertIn(b"RAWBYTES", body)
        self.assertTrue(body.rstrip().endswith(f"--{boundary}--".encode()))

    def test_stt_parses_transcription(self) -> None:
        body = json.dumps({"text": "  which motor is hottest  "}).encode("utf-8")
        with mock.patch.object(server, "LLM_STT_ENABLED", True), mock.patch(
            "urllib.request.urlopen", return_value=_fake_response(body)
        ):
            status, response = server.transcribe_audio(b"audio-bytes", "audio/webm")
        self.assertEqual(status, 200)
        self.assertEqual(response["text"], "which motor is hottest")

    def test_stt_rejects_oversize_audio(self) -> None:
        with mock.patch.object(server, "LLM_STT_ENABLED", True), mock.patch.object(
            server, "MAX_AUDIO_BYTES", 4
        ):
            status, response = server.transcribe_audio(b"toolong", "audio/webm")
        self.assertEqual(status, 413)

    def test_tts_returns_audio_bytes(self) -> None:
        with mock.patch.object(server, "LLM_TTS_ENABLED", True), mock.patch(
            "urllib.request.urlopen", return_value=_fake_response(b"ID3AUDIO", "audio/mpeg")
        ):
            status, result, ctype = server.synthesize_speech("hello")
        self.assertEqual(status, 200)
        self.assertEqual(result, b"ID3AUDIO")
        self.assertEqual(ctype, "audio/mpeg")

    def test_tts_rejects_empty_text(self) -> None:
        with mock.patch.object(server, "LLM_TTS_ENABLED", True):
            status, result, ctype = server.synthesize_speech("   ")
        self.assertEqual(status, 400)


class _fake_response:
    def __init__(self, body: bytes, content_type: str = "application/json") -> None:
        self._body = body
        self.headers = {"Content-Type": content_type}

    def read(self) -> bytes:
        return self._body

    def __enter__(self):
        return self

    def __exit__(self, *args) -> None:
        return None


if __name__ == "__main__":
    unittest.main()
