from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

import claude_bridge


TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "propose_arm_pose",
            "description": "Plan an arm pose.",
            "parameters": {"type": "object", "properties": {"joints": {"type": "object"}}, "required": []},
        },
    }
]


class PromptBuildingTest(unittest.TestCase):
    def test_system_prompt_carries_tool_specs_and_protocol(self) -> None:
        system = claude_bridge.build_system_prompt("BASE SYSTEM", TOOLS)
        self.assertIn("BASE SYSTEM", system)
        self.assertIn("propose_arm_pose", system)
        self.assertIn('"tool_call"', system)

    def test_transcript_flattens_all_roles_in_order(self) -> None:
        messages = [
            {"role": "system", "content": "SYS"},
            {"role": "user", "content": "raise your hand"},
            {"role": "assistant", "content": "", "tool_calls": [
                {"id": "c1", "function": {"name": "propose_arm_pose", "arguments": "{\"joints\": {}}"}}
            ]},
            {"role": "tool", "tool_call_id": "c1", "content": "{\"ok\": true}"},
            {"role": "assistant", "content": "Check the preview."},
            {"role": "user", "content": "okay"},
        ]
        system, transcript = claude_bridge.split_messages(messages)
        self.assertEqual(system, "SYS")
        self.assertNotIn("SYS", transcript)
        for token in ("Operator: raise your hand", "propose_arm_pose", '{"ok": true}',
                      "Assistant: Check the preview.", "Operator: okay"):
            self.assertIn(token, transcript)
        self.assertLess(transcript.index("raise your hand"), transcript.index("okay"))

    def test_transcript_handles_structured_content_blocks(self) -> None:
        messages = [{"role": "user", "content": [{"type": "text", "text": "hello"},
                                                  {"type": "image_url", "image_url": {"url": "data:..."}}]}]
        _, transcript = claude_bridge.split_messages(messages)
        self.assertIn("hello", transcript)
        self.assertNotIn("data:", transcript)


class VisionPathTest(unittest.TestCase):
    def test_extract_images_maps_openai_blocks_to_anthropic(self) -> None:
        messages = [{"role": "user", "content": [
            {"type": "text", "text": "copy this"},
            {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64,QUJD"}},
        ]}]
        images = claude_bridge.extract_images(messages)
        self.assertEqual(len(images), 1)
        self.assertEqual(images[0], {
            "type": "image",
            "source": {"type": "base64", "media_type": "image/jpeg", "data": "QUJD"},
        })

    def test_extract_images_ignores_text_and_malformed_urls(self) -> None:
        messages = [
            {"role": "user", "content": "plain string, no blocks"},
            {"role": "user", "content": [
                {"type": "text", "text": "hi"},
                {"type": "image_url", "image_url": {"url": "https://example.com/x.png"}},
                {"type": "image_url", "image_url": {"url": "data:image/png;notbase64,zzz"}},
            ]},
        ]
        self.assertEqual(claude_bridge.extract_images(messages), [])

    def test_stream_command_uses_stream_json_in_and_out(self) -> None:
        cmd = claude_bridge.claude_command_stream("SYSTEM")
        joined = " ".join(cmd)
        self.assertIn("--input-format stream-json", joined)
        self.assertIn("--output-format stream-json", joined)
        self.assertIn("--tools", cmd)  # same isolation guards as the text path
        self.assertIn("--no-session-persistence", cmd)

    def test_last_stream_result_returns_terminal_result_object(self) -> None:
        stdout = (
            '{"type":"system","subtype":"init"}\n'
            '{"type":"assistant","message":{"content":[{"type":"text","text":"Magenta"}]}}\n'
            '{"type":"result","is_error":false,"result":"Magenta"}\n'
        )
        obj = claude_bridge._last_stream_result(stdout)
        self.assertEqual(obj["result"], "Magenta")
        self.assertFalse(obj["is_error"])

    def test_last_stream_result_none_when_absent(self) -> None:
        self.assertIsNone(claude_bridge._last_stream_result('{"type":"system"}\nnoise\n'))


class ReplyParsingTest(unittest.TestCase):
    def test_plain_text_becomes_openai_message(self) -> None:
        response = claude_bridge.openai_response_from_result("Hello there.", "claude-sonnet")
        message = response["choices"][0]["message"]
        self.assertEqual(message["content"], "Hello there.")
        self.assertNotIn("tool_calls", message)

    def test_bare_tool_call_json_maps_to_tool_calls(self) -> None:
        raw = '{"tool_call": {"name": "propose_arm_pose", "arguments": {"joints": {"RightElbow": 1.5}}}}'
        response = claude_bridge.openai_response_from_result(raw, "claude-sonnet")
        calls = response["choices"][0]["message"]["tool_calls"]
        self.assertEqual(calls[0]["function"]["name"], "propose_arm_pose")
        self.assertEqual(json.loads(calls[0]["function"]["arguments"])["joints"]["RightElbow"], 1.5)

    def test_fenced_tool_call_json_is_recovered(self) -> None:
        raw = 'Sure.\n```json\n{"tool_call": {"name": "move", "arguments": {"position": "proposed", "confirm": true}}}\n```'
        response = claude_bridge.openai_response_from_result(raw, "m")
        calls = response["choices"][0]["message"]["tool_calls"]
        self.assertEqual(calls[0]["function"]["name"], "move")

    def test_prose_wrapped_bare_tool_call_is_recovered(self) -> None:
        raw = 'Sure, calling the tool now:\n{"tool_call": {"name": "move", "arguments": {"position": "proposed"}}} done'
        response = claude_bridge.openai_response_from_result(raw, "m")
        message = response["choices"][0]["message"]
        self.assertEqual(message["tool_calls"][0]["function"]["name"], "move")
        self.assertEqual(response["choices"][0]["finish_reason"], "tool_calls")

    def test_plain_text_finish_reason_is_stop(self) -> None:
        response = claude_bridge.openai_response_from_result("just chatting", "m")
        self.assertEqual(response["choices"][0]["finish_reason"], "stop")

    def test_already_string_arguments_not_double_encoded(self) -> None:
        raw = '{"tool_call": {"name": "move", "arguments": "{\\"position\\": \\"home\\"}"}}'
        response = claude_bridge.openai_response_from_result(raw, "m")
        args = response["choices"][0]["message"]["tool_calls"][0]["function"]["arguments"]
        self.assertEqual(json.loads(args)["position"], "home")

    def test_lenient_json_object_skips_leading_noise(self) -> None:
        decoded = claude_bridge._first_json_object('(node warning)\n{"is_error": false, "result": "hi"}')
        self.assertEqual(decoded["result"], "hi")

    def test_command_line_is_isolated_and_tool_free(self) -> None:
        cmd = claude_bridge.claude_command("SYSTEM")
        joined = " ".join(cmd)
        self.assertIn("--output-format json", joined)
        self.assertIn("--tools", cmd)
        self.assertIn("--setting-sources", cmd)
        self.assertIn("--no-session-persistence", cmd)


if __name__ == "__main__":
    unittest.main()
