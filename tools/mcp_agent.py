#!/usr/bin/env python3
"""Agent CLI that drives the dashboard's MCP endpoint with the on-prem LLM.

Connects two things that already exist:
  * the robot's MCP endpoint (POST /mcp, see README "MCP Endpoint") — the
    stable, guarded tool interface, and
  * the on-prem OpenAI-compatible LLM (Ollama on the AI host) — the reasoning.

The loop discovers tools via MCP tools/list, offers them to the model as
OpenAI-style function specs, executes requested calls via MCP tools/call, and
feeds results back until the model produces a final answer. Tool safety lives
server-side (confirm gates, name validation), never in this client.

Usage:
  python3 tools/mcp_agent.py                         # interactive REPL
  python3 tools/mcp_agent.py -p "check loco status"  # one-shot
Stdlib only, like the rest of the repo.
"""

from __future__ import annotations

import argparse
import json
import sys
import urllib.error
import urllib.request
from typing import Any

DEFAULT_MCP_URL = "http://10.2.100.142:8088/mcp"
DEFAULT_LLM_URL = "http://10.2.125.3:11434"
DEFAULT_MODEL = "qwen3:30b-a3b-instruct-2507-q4_K_M"
MCP_PROTOCOL_VERSION = "2025-06-18"
MAX_TOOL_ROUNDS = 6
HTTP_TIMEOUT_SECONDS = 120.0

SYSTEM_PROMPT = (
    "You are an operator assistant for a Unitree H1-2 humanoid robot. Use the "
    "available tools to answer questions about the robot's live state and, when "
    "the operator's latest message explicitly asks for it, perform guarded "
    "actions (e.g. the move tool drives the arms to a saved named position). "
    "Operators may write in any language (English, Turkish, German, ...): map "
    "their request to the closest position name the move tool lists and pass "
    "confirm=true. Follow the tool descriptions and the server instructions "
    "exactly. Reply in short plain sentences, in the operator's language, with "
    "the specific facts you found; say so plainly when a tool reports missing "
    "data."
)


def post_json(url: str, payload: dict[str, Any], headers: dict[str, str]) -> dict[str, Any]:
    body = json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        url, data=body, headers={"Content-Type": "application/json", **headers}, method="POST"
    )
    with urllib.request.urlopen(request, timeout=HTTP_TIMEOUT_SECONDS) as response:
        raw = response.read()
    if not raw:  # 202 for MCP notifications
        return {}
    decoded = json.loads(raw.decode("utf-8"))
    if not isinstance(decoded, dict):
        raise ValueError(f"Expected a JSON object from {url}")
    return decoded


class McpClient:
    """Minimal stateless streamable-HTTP MCP client (JSON responses only)."""

    def __init__(self, url: str, token: str = "") -> None:
        self.url = url
        self.headers = {"MCP-Protocol-Version": MCP_PROTOCOL_VERSION}
        if token:
            self.headers["Authorization"] = f"Bearer {token}"
        self._next_id = 0

    def _rpc(self, method: str, params: dict[str, Any] | None = None,
             notification: bool = False) -> dict[str, Any]:
        message: dict[str, Any] = {"jsonrpc": "2.0", "method": method}
        if params is not None:
            message["params"] = params
        if not notification:
            self._next_id += 1
            message["id"] = self._next_id
        response = post_json(self.url, message, self.headers)
        if notification:
            return {}
        error = response.get("error")
        if error:
            raise RuntimeError(f"MCP {method} failed: {error.get('message', error)}")
        result = response.get("result")
        if not isinstance(result, dict):
            raise RuntimeError(f"MCP {method} returned no result object")
        return result

    def initialize(self) -> dict[str, Any]:
        result = self._rpc("initialize", {
            "protocolVersion": MCP_PROTOCOL_VERSION,
            "capabilities": {},
            "clientInfo": {"name": "mcp_agent", "version": "1.0.0"},
        })
        self._rpc("notifications/initialized", notification=True)
        return result

    def list_tools(self) -> list[dict[str, Any]]:
        tools = self._rpc("tools/list").get("tools")
        return tools if isinstance(tools, list) else []

    def call_tool(self, name: str, arguments: dict[str, Any]) -> str:
        result = self._rpc("tools/call", {"name": name, "arguments": arguments})
        texts = [
            block.get("text", "")
            for block in result.get("content", [])
            if isinstance(block, dict) and block.get("type") == "text"
        ]
        return "\n".join(texts) or json.dumps(result)


def to_openai_specs(mcp_tools: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """MCP tool descriptors -> OpenAI-style function specs (same JSON Schema)."""
    return [
        {
            "type": "function",
            "function": {
                "name": tool["name"],
                "description": tool.get("description", ""),
                "parameters": tool.get("inputSchema", {"type": "object", "properties": {}}),
            },
        }
        for tool in mcp_tools
    ]


def call_llm(llm_url: str, model: str, messages: list[dict[str, Any]],
             tools: list[dict[str, Any]]) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "model": model,
        "messages": messages,
        "stream": False,
        "temperature": 0.2,
    }
    if tools:
        payload["tools"] = tools
    response = post_json(f"{llm_url.rstrip('/')}/v1/chat/completions", payload, {})
    message = response["choices"][0]["message"]
    if not isinstance(message, dict):
        raise ValueError("LLM response message is not an object")
    return message


def parse_tool_arguments(raw: Any) -> dict[str, Any]:
    if isinstance(raw, dict):
        return raw
    if isinstance(raw, str) and raw.strip():
        parsed = json.loads(raw)
        if isinstance(parsed, dict):
            return parsed
    return {}


def run_turn(client: McpClient, llm_url: str, model: str,
             messages: list[dict[str, Any]], tools: list[dict[str, Any]]) -> str:
    """One user turn: loop LLM <-> MCP tool calls until a final text reply."""
    for _ in range(MAX_TOOL_ROUNDS):
        message = call_llm(llm_url, model, messages, tools)
        calls = message.get("tool_calls")
        if not calls:
            reply = message.get("content") or ""
            messages.append({"role": "assistant", "content": reply})
            return reply
        messages.append({"role": "assistant", "content": message.get("content") or "",
                         "tool_calls": calls})
        for call in calls:
            function = call.get("function") if isinstance(call, dict) else None
            function = function if isinstance(function, dict) else {}
            name = str(function.get("name") or "")
            try:
                arguments = parse_tool_arguments(function.get("arguments"))
                print(f"  [tool] {name}({json.dumps(arguments)})", file=sys.stderr)
                result_text = client.call_tool(name, arguments)
            except Exception as exc:
                result_text = json.dumps({"ok": False, "error": str(exc)})
            messages.append({
                "role": "tool",
                "tool_call_id": str((call.get("id") if isinstance(call, dict) else None) or name),
                "content": result_text,
            })
    # Rounds exhausted: force a final answer without tools.
    message = call_llm(llm_url, model, messages, [])
    reply = message.get("content") or ""
    messages.append({"role": "assistant", "content": reply})
    return reply


def main() -> int:
    parser = argparse.ArgumentParser(description="Drive the robot's MCP tools with the on-prem LLM")
    parser.add_argument("--mcp-url", default=DEFAULT_MCP_URL)
    parser.add_argument("--token", default="", help="Bearer token if the endpoint sets MCP_TOKEN")
    parser.add_argument("--llm-url", default=DEFAULT_LLM_URL)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("-p", "--prompt", default="", help="One-shot prompt (otherwise REPL)")
    args = parser.parse_args()

    client = McpClient(args.mcp_url, args.token)
    try:
        init = client.initialize()
    except (urllib.error.URLError, RuntimeError, ValueError) as exc:
        print(f"Cannot initialize MCP at {args.mcp_url}: {exc}", file=sys.stderr)
        return 1
    tools = to_openai_specs(client.list_tools())
    server_name = init.get("serverInfo", {}).get("name", "mcp")
    print(f"Connected to {server_name}: {len(tools)} tools, model {args.model}", file=sys.stderr)

    system = SYSTEM_PROMPT
    instructions = init.get("instructions")
    if isinstance(instructions, str) and instructions:
        system = f"{system}\n\nServer instructions:\n{instructions}"
    messages: list[dict[str, Any]] = [{"role": "system", "content": system}]

    if args.prompt:
        messages.append({"role": "user", "content": args.prompt})
        print(run_turn(client, args.llm_url, args.model, messages, tools))
        return 0

    while True:
        try:
            line = input("you> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return 0
        if not line or line.lower() in ("exit", "quit"):
            return 0
        messages.append({"role": "user", "content": line})
        print(run_turn(client, args.llm_url, args.model, messages, tools))


if __name__ == "__main__":
    sys.exit(main())
