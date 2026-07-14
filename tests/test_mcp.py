"""MCP endpoint tests: JSON-RPC dispatch, tool exposure, and — critically —
that every guard the chat tools enforce (chill confirm gate, ros2 name
validation, feature flags) holds identically for MCP clients."""

from __future__ import annotations

import json
import unittest
from unittest import mock
from tempfile import TemporaryDirectory

import server


def rpc(method: str, params: dict | None = None, request_id: int | str | None = 1) -> dict:
    message: dict = {"jsonrpc": "2.0", "method": method}
    if request_id is not None:
        message["id"] = request_id
    if params is not None:
        message["params"] = params
    return message


class McpDescriptorTest(unittest.TestCase):
    def test_specs_map_to_mcp_shape(self) -> None:
        tools = server.mcp_tool_descriptors(server.CHAT_TOOL_SPECS)
        self.assertEqual(len(tools), len(server.CHAT_TOOL_SPECS))
        for tool in tools:
            self.assertEqual(sorted(tool), ["description", "inputSchema", "name"])
            self.assertEqual(tool["inputSchema"]["type"], "object")


class McpRequestTest(unittest.TestCase):
    def setUp(self) -> None:
        self.store = server.TelemetryStore(domain=0, robot_host="127.0.0.1")

    def test_rejects_non_jsonrpc_payloads(self) -> None:
        for payload in (None, [], "hi", {"method": "ping", "id": 1}):
            response = self.store.mcp_request(payload)
            self.assertEqual(response["error"]["code"], -32600)

    def test_notifications_get_no_response(self) -> None:
        self.assertIsNone(
            self.store.mcp_request(rpc("notifications/initialized", request_id=None))
        )

    def test_initialize_negotiates_protocol_version(self) -> None:
        response = self.store.mcp_request(rpc("initialize", {"protocolVersion": "2025-03-26"}))
        self.assertEqual(response["result"]["protocolVersion"], "2025-03-26")
        self.assertIn("tools", response["result"]["capabilities"])
        response = self.store.mcp_request(rpc("initialize", {"protocolVersion": "1999-01-01"}))
        self.assertEqual(response["result"]["protocolVersion"], server.MCP_PROTOCOL_VERSIONS[0])

    def test_ping_and_unknown_method(self) -> None:
        self.assertEqual(self.store.mcp_request(rpc("ping"))["result"], {})
        response = self.store.mcp_request(rpc("resources/list"))
        self.assertEqual(response["error"]["code"], -32601)

    def test_tools_list_matches_chat_tools(self) -> None:
        # named_positions() reads the real recordings dir; pin it empty so the
        # dynamic move tool is absent regardless of local saved poses.
        with TemporaryDirectory() as directory:
            with mock.patch.object(server, "RECORDINGS_DIR", server.Path(directory)):
                names = [t["name"] for t in self.store.mcp_request(rpc("tools/list"))["result"]["tools"]]
        self.assertEqual(
            names, [spec["function"]["name"] for spec in server.CHAT_TOOL_SPECS]
        )

    def test_tools_list_gains_move_when_named_position_exists(self) -> None:
        with TemporaryDirectory() as directory:
            (server.Path(directory) / "20260714-111543-home.pose.json").write_text("{}", encoding="utf-8")
            with mock.patch.object(server, "RECORDINGS_DIR", server.Path(directory)):
                tools = self.store.mcp_request(rpc("tools/list"))["result"]["tools"]
        move = next(t for t in tools if t["name"] == "move")
        self.assertEqual(move["inputSchema"]["properties"]["position"]["enum"], ["home"])

    def test_tools_list_hides_chill_when_disabled(self) -> None:
        with mock.patch.object(server, "LLM_TOOL_CHILL_ENABLED", False):
            names = [t["name"] for t in self.store.mcp_request(rpc("tools/list"))["result"]["tools"]]
        self.assertNotIn("chill_motors", names)

    def test_tools_call_dispatches_through_run_chat_tool(self) -> None:
        with mock.patch.object(
            self.store, "run_chat_tool", return_value={"ok": True, "loco": {}}
        ) as run:
            response = self.store.mcp_request(rpc("tools/call", {"name": "get_loco_status"}))
        run.assert_called_once_with("get_loco_status", {})
        result = response["result"]
        self.assertFalse(result["isError"])
        self.assertEqual(json.loads(result["content"][0]["text"]), {"ok": True, "loco": {}})

    def test_tools_call_unknown_tool_is_invalid_params(self) -> None:
        response = self.store.mcp_request(rpc("tools/call", {"name": "launch_rockets"}))
        self.assertEqual(response["error"]["code"], -32602)

    def test_tools_call_rejects_non_object_arguments(self) -> None:
        response = self.store.mcp_request(
            rpc("tools/call", {"name": "get_loco_status", "arguments": [1]})
        )
        self.assertEqual(response["error"]["code"], -32602)

    def test_chill_hidden_from_call_when_disabled(self) -> None:
        with mock.patch.object(server, "LLM_TOOL_CHILL_ENABLED", False):
            response = self.store.mcp_request(
                rpc("tools/call", {"name": "chill_motors", "arguments": {"confirm": True}})
            )
        self.assertEqual(response["error"]["code"], -32602)

    def test_chill_confirm_gate_holds_over_mcp(self) -> None:
        with mock.patch.object(self.store, "chill_motors") as chill:
            response = self.store.mcp_request(
                rpc("tools/call", {"name": "chill_motors", "arguments": {}})
            )
        chill.assert_not_called()
        result = response["result"]
        self.assertTrue(result["isError"])
        self.assertIn("confirm", result["content"][0]["text"])

    def test_chill_with_confirm_executes(self) -> None:
        with mock.patch.object(
            self.store, "chill_motors", return_value=(200, {"ok": True})
        ) as chill:
            response = self.store.mcp_request(
                rpc("tools/call", {"name": "chill_motors", "arguments": {"confirm": True}})
            )
        chill.assert_called_once()
        self.assertFalse(response["result"]["isError"])

    def test_ros2_name_validation_holds_over_mcp(self) -> None:
        response = self.store.mcp_request(
            rpc("tools/call", {"name": "ros2_node_info", "arguments": {"node": "--help; rm -rf"}})
        )
        result = response["result"]
        self.assertTrue(result["isError"])
        self.assertIn("Invalid node name", result["content"][0]["text"])


if __name__ == "__main__":
    unittest.main()
