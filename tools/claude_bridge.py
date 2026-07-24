"""Claude Code bridge: OpenAI-compatible chat completions served by the local CLI.

Run this on the operator's Mac (the machine where `claude` is installed and
logged in). It exposes POST /v1/chat/completions on the LAN; the robot
dashboard forwards its UNCHANGED system prompt, message history and tool specs
here when the operator flips the chat backend to Claude, so Claude answers
under exactly the same context and guardrails as the default qwen backend.

    python3 tools/claude_bridge.py [--host 0.0.0.0] [--port 8399]

Then point the robot at it:  CLAUDE_BRIDGE_URL=http://<mac-lan-ip>:8399

No Anthropic API key is needed — requests run through the local Claude Code
CLI (non-interactive `claude -p`, all CLI tools disabled, user settings not
loaded, no session persistence). Tool use is bridged textually: the specs are
embedded in the system prompt and Claude answers a bare
{"tool_call": {"name": ..., "arguments": {...}}} JSON object, which this
bridge maps back to an OpenAI-style tool_calls response for the dashboard's
guarded tool loop.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

CLAUDE_BIN = os.environ.get("CLAUDE_BRIDGE_CLI", shutil.which("claude") or
                            os.path.expanduser("~/.local/bin/claude"))
CLAUDE_MODEL = os.environ.get("CLAUDE_BRIDGE_CLAUDE_MODEL", "opus")
CLAUDE_TIMEOUT_SECONDS = float(os.environ.get("CLAUDE_BRIDGE_TIMEOUT", "180"))
MAX_BODY_BYTES = 2_000_000
# Cap concurrent CLI subprocesses: this listens on the LAN, and each request
# forks a billed `claude` process. Excess requests get 503 rather than swamping
# the Mac / the operator's rate limit.
MAX_CONCURRENCY = max(1, int(os.environ.get("CLAUDE_BRIDGE_MAX_CONCURRENCY", "3")))
_SEM = threading.BoundedSemaphore(MAX_CONCURRENCY)
# Optional shared secret. When set, POSTs must carry Authorization: Bearer <it>.
CLAUDE_BRIDGE_TOKEN = os.environ.get("CLAUDE_BRIDGE_TOKEN", "")

TOOL_PROTOCOL = (
    "\n\nTOOLS: you can call the tools below. To call one, reply with ONLY this bare JSON "
    "object and NOTHING else (no prose, no markdown fence):\n"
    '{"tool_call": {"name": "<tool name>", "arguments": {...}}}\n'
    "The result will come back as a 'Tool result' message; then continue. When no tool is "
    "needed, answer normally in plain text. Available tools (JSON Schema):\n"
)


def build_system_prompt(system: str, tools: list[dict[str, Any]] | None) -> str:
    if not tools:
        return system
    specs = json.dumps([t.get("function", {}) for t in tools], ensure_ascii=False, indent=1)
    return system + TOOL_PROTOCOL + specs


def _content_text(content: Any) -> str:
    """OpenAI content may be a string or a list of typed blocks (vision)."""
    if isinstance(content, str):
        return content
    if isinstance(content, list):
        return " ".join(
            block.get("text", "") for block in content
            if isinstance(block, dict) and block.get("type") == "text"
        )
    return ""


def split_messages(messages: list[dict[str, Any]]) -> tuple[str, str]:
    """(system prompt, flattened conversation transcript for stdin)."""
    system_parts: list[str] = []
    lines: list[str] = []
    for message in messages:
        role = message.get("role")
        text = _content_text(message.get("content"))
        if role == "system":
            system_parts.append(text)
        elif role == "user":
            lines.append(f"Operator: {text}")
        elif role == "assistant":
            for call in message.get("tool_calls") or []:
                function = call.get("function") or {}
                lines.append(
                    f"Assistant called tool {function.get('name')} with arguments {function.get('arguments')}"
                )
            if text:
                lines.append(f"Assistant: {text}")
        elif role == "tool":
            lines.append(f"Tool result: {text}")
    lines.append("Assistant:")
    return "\n".join(system_parts), "\n\n".join(lines)


def claude_command(system: str) -> list[str]:
    return [
        CLAUDE_BIN, "-p",
        "--output-format", "json",
        "--tools", "",                 # no local CLI tools: the dashboard runs OUR tools
        "--setting-sources", "",       # do not load the operator's personal settings/hooks
        "--no-session-persistence",
        "--model", CLAUDE_MODEL,
        "--system-prompt", system,
    ]


def _first_json_object(text: str) -> Any:
    """Decode the first standalone JSON object embedded anywhere in text.

    Claude often narrates ("Sure: {...}") or fences the tool-call JSON; scanning
    each '{' with raw_decode recovers it whether or not it stands alone."""
    decoder = json.JSONDecoder()
    index = text.find("{")
    while index != -1:
        try:
            parsed, _ = decoder.raw_decode(text, index)
        except ValueError:
            parsed = None
        if parsed is not None:
            return parsed
        index = text.find("{", index + 1)
    return None


def _extract_tool_call(text: str) -> dict[str, Any] | None:
    decoder = json.JSONDecoder()
    index = text.find("{")
    while index != -1:
        try:
            parsed, _ = decoder.raw_decode(text, index)
        except ValueError:
            parsed = None
        if isinstance(parsed, dict):
            call = parsed.get("tool_call")
            if isinstance(call, dict) and isinstance(call.get("name"), str):
                return call
        index = text.find("{", index + 1)
    return None


def _arguments_json(arguments: Any) -> str:
    # OpenAI function.arguments is a JSON STRING. Pass a model-provided string
    # through unchanged (avoid double-encoding); serialize anything else.
    if isinstance(arguments, str):
        return arguments
    return json.dumps(arguments or {}, ensure_ascii=False)


def openai_response_from_result(result_text: str, model: str) -> dict[str, Any]:
    message: dict[str, Any] = {"role": "assistant", "content": result_text}
    finish_reason = "stop"
    call = _extract_tool_call(result_text)
    if call is not None:
        finish_reason = "tool_calls"  # spec-conformant clients branch on this
        message = {
            "role": "assistant",
            "content": "",
            "tool_calls": [{
                "id": f"bridge-{time.monotonic_ns()}",
                "type": "function",
                "function": {
                    "name": call["name"],
                    "arguments": _arguments_json(call.get("arguments")),
                },
            }],
        }
    return {
        "id": f"claude-bridge-{time.monotonic_ns()}",
        "object": "chat.completion",
        "model": model,
        "choices": [{"index": 0, "message": message, "finish_reason": finish_reason}],
        "usage": {},
    }


def run_claude(messages: list[dict[str, Any]], tools: list[dict[str, Any]] | None) -> dict[str, Any]:
    system, transcript = split_messages(messages)
    completed = subprocess.run(
        claude_command(build_system_prompt(system, tools)),
        input=transcript, capture_output=True, text=True, timeout=CLAUDE_TIMEOUT_SECONDS,
    )
    if completed.returncode != 0:
        raise RuntimeError(f"claude CLI failed: {completed.stderr.strip()[:300]}")
    # Tolerate leading noise (node deprecation warnings, etc.) on stdout.
    decoded = _first_json_object(completed.stdout)
    if not isinstance(decoded, dict):
        raise RuntimeError(f"claude produced no JSON result: {completed.stdout.strip()[:200]!r}")
    if decoded.get("is_error"):
        raise RuntimeError(f"claude reported an error: {str(decoded.get('result'))[:300]}")
    model = CLAUDE_MODEL
    usage = decoded.get("modelUsage")
    if isinstance(usage, dict) and usage:
        # Claude Code lists every model it touched (haiku handles background
        # chores); the MAIN model is the one that produced the most output.
        def output_tokens(item: tuple[str, Any]) -> int:
            stats = item[1] if isinstance(item[1], dict) else {}
            value = stats.get("outputTokens")
            return value if isinstance(value, int) else 0

        model = max(usage.items(), key=output_tokens)[0]
    return openai_response_from_result(str(decoded.get("result") or ""), model)


class BridgeHandler(BaseHTTPRequestHandler):
    def _send_json(self, payload: dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        if self.path == "/health":
            self._send_json({"ok": True, "backend": "claude-code", "model": CLAUDE_MODEL})
        else:
            self._send_json({"ok": False, "error": "Unknown path."}, HTTPStatus.NOT_FOUND)

    def _authorized(self) -> bool:
        if not CLAUDE_BRIDGE_TOKEN:
            return True
        return self.headers.get("Authorization", "") == f"Bearer {CLAUDE_BRIDGE_TOKEN}"

    def do_POST(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        if self.path != "/v1/chat/completions":
            self._send_json({"ok": False, "error": "Unknown path."}, HTTPStatus.NOT_FOUND)
            return
        if not self._authorized():
            self._send_json({"error": "Unauthorized."}, HTTPStatus.UNAUTHORIZED)
            return
        length = int(self.headers.get("Content-Length") or 0)
        if length <= 0 or length > MAX_BODY_BYTES:
            self._send_json({"error": "Bad Content-Length."}, HTTPStatus.BAD_REQUEST)
            return
        try:
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
            messages = payload["messages"]
            if not isinstance(messages, list):
                raise ValueError("messages must be a list")
        except (ValueError, KeyError) as exc:
            self._send_json({"error": f"Invalid request: {exc}"}, HTTPStatus.BAD_REQUEST)
            return
        # Bound concurrent CLI subprocesses; shed load instead of swamping the Mac.
        if not _SEM.acquire(timeout=2):
            self._send_json({"error": "Bridge busy; try again."}, HTTPStatus.SERVICE_UNAVAILABLE)
            return
        try:
            response = run_claude(messages, payload.get("tools"))
        except subprocess.TimeoutExpired:
            self._send_json({"error": "claude CLI timed out."}, HTTPStatus.GATEWAY_TIMEOUT)
            return
        except Exception as exc:  # surface CLI failures to the dashboard verbatim
            self._send_json({"error": str(exc)}, HTTPStatus.BAD_GATEWAY)
            return
        finally:
            _SEM.release()
        self._send_json(response)

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f"[bridge] {self.address_string()} {fmt % args}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Claude Code chat-completions bridge")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8399)
    args = parser.parse_args()
    if not CLAUDE_BIN or not os.path.exists(CLAUDE_BIN):
        raise SystemExit("claude CLI not found — install Claude Code or set CLAUDE_BRIDGE_CLI.")
    server = ThreadingHTTPServer((args.host, args.port), BridgeHandler)
    print(f"Claude bridge listening on {args.host}:{args.port} (model {CLAUDE_MODEL}, cli {CLAUDE_BIN})")
    server.serve_forever()


if __name__ == "__main__":
    main()
