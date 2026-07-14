"""Tests for tools/mcp_agent.py: spec conversion, the MCP client's JSON-RPC
framing, and the LLM<->tool loop (all network calls mocked)."""

from __future__ import annotations

import importlib.util
import json
import unittest
from pathlib import Path
from unittest import mock

_SPEC = importlib.util.spec_from_file_location(
    "mcp_agent", Path(__file__).resolve().parent.parent / "tools" / "mcp_agent.py"
)
mcp_agent = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(mcp_agent)


MCP_TOOL = {
    "name": "get_joint_details",
    "description": "Full live state of one joint.",
    "inputSchema": {"type": "object", "properties": {"joint": {"type": "string"}},
                    "required": ["joint"]},
}


class SpecConversionTest(unittest.TestCase):
    def test_mcp_descriptor_maps_to_openai_spec(self) -> None:
        specs = mcp_agent.to_openai_specs([MCP_TOOL])
        self.assertEqual(specs, [{
            "type": "function",
            "function": {
                "name": "get_joint_details",
                "description": "Full live state of one joint.",
                "parameters": MCP_TOOL["inputSchema"],
            },
        }])

    def test_missing_schema_gets_empty_object_schema(self) -> None:
        specs = mcp_agent.to_openai_specs([{"name": "ping"}])
        self.assertEqual(specs[0]["function"]["parameters"]["type"], "object")


class McpClientTest(unittest.TestCase):
    def test_initialize_sends_handshake_then_notification(self) -> None:
        client = mcp_agent.McpClient("http://robot/mcp", token="secret")
        responses = [
            {"jsonrpc": "2.0", "id": 1, "result": {"protocolVersion": "2025-06-18"}},
            {},  # 202 for notifications/initialized
        ]
        with mock.patch.object(mcp_agent, "post_json", side_effect=responses) as post:
            result = client.initialize()
        self.assertEqual(result["protocolVersion"], "2025-06-18")
        first, second = post.call_args_list
        self.assertEqual(first.args[1]["method"], "initialize")
        self.assertEqual(first.args[2]["Authorization"], "Bearer secret")
        self.assertEqual(second.args[1]["method"], "notifications/initialized")
        self.assertNotIn("id", second.args[1])

    def test_rpc_error_raises(self) -> None:
        client = mcp_agent.McpClient("http://robot/mcp")
        error = {"jsonrpc": "2.0", "id": 1, "error": {"code": -32602, "message": "Unknown tool"}}
        with mock.patch.object(mcp_agent, "post_json", return_value=error):
            with self.assertRaisesRegex(RuntimeError, "Unknown tool"):
                client.call_tool("nope", {})

    def test_call_tool_joins_text_content(self) -> None:
        client = mcp_agent.McpClient("http://robot/mcp")
        response = {"jsonrpc": "2.0", "id": 1, "result": {
            "content": [{"type": "text", "text": '{"ok": true}'}], "isError": False}}
        with mock.patch.object(mcp_agent, "post_json", return_value=response):
            self.assertEqual(client.call_tool("get_loco_status", {}), '{"ok": true}')


class RunTurnTest(unittest.TestCase):
    def test_tool_call_is_executed_and_fed_back(self) -> None:
        client = mcp_agent.McpClient("http://robot/mcp")
        llm_replies = [
            {"content": "", "tool_calls": [{
                "id": "call_1",
                "function": {"name": "get_loco_status", "arguments": '{}'},
            }]},
            {"content": "The robot is idle."},
        ]
        with mock.patch.object(mcp_agent, "call_llm", side_effect=llm_replies) as llm, \
                mock.patch.object(client, "call_tool", return_value='{"ok": true, "loco": {}}') as tool:
            messages = [{"role": "system", "content": "s"}, {"role": "user", "content": "u"}]
            reply = mcp_agent.run_turn(client, "http://llm", "m", messages, [])
        self.assertEqual(reply, "The robot is idle.")
        tool.assert_called_once_with("get_loco_status", {})
        tool_message = llm.call_args_list[1].args[2][-2]
        self.assertEqual(tool_message["role"], "tool")
        self.assertEqual(json.loads(tool_message["content"]), {"ok": True, "loco": {}})

    def test_rounds_exhausted_forces_final_answer_without_tools(self) -> None:
        client = mcp_agent.McpClient("http://robot/mcp")
        tool_call_reply = {"content": "", "tool_calls": [{
            "id": "x", "function": {"name": "get_loco_status", "arguments": "{}"}}]}
        llm_replies = [tool_call_reply] * mcp_agent.MAX_TOOL_ROUNDS + [{"content": "done"}]
        with mock.patch.object(mcp_agent, "call_llm", side_effect=llm_replies) as llm, \
                mock.patch.object(client, "call_tool", return_value="{}"):
            reply = mcp_agent.run_turn(client, "http://llm", "m", [], [MCP_TOOL])
        self.assertEqual(reply, "done")
        self.assertEqual(llm.call_args_list[-1].args[3], [])  # final call offers no tools

    def test_tool_failure_is_reported_not_raised(self) -> None:
        client = mcp_agent.McpClient("http://robot/mcp")
        llm_replies = [
            {"content": "", "tool_calls": [{
                "id": "x", "function": {"name": "get_loco_status", "arguments": "{}"}}]},
            {"content": "sorry"},
        ]
        with mock.patch.object(mcp_agent, "call_llm", side_effect=llm_replies) as llm, \
                mock.patch.object(client, "call_tool", side_effect=RuntimeError("boom")):
            reply = mcp_agent.run_turn(client, "http://llm", "m", [], [])
        self.assertEqual(reply, "sorry")
        tool_message = llm.call_args_list[1].args[2][-2]
        self.assertIn("boom", tool_message["content"])


if __name__ == "__main__":
    unittest.main()
