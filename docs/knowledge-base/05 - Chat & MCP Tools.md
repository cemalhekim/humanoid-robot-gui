---
tags: [llm, chat, mcp, tools, ollama, feature-flags]
summary: The on-prem LLM Command Center assistant, its guarded tool system, the MCP endpoint, and every LLM_TOOL_* / MCP feature flag.
---

# 05 - Chat & MCP Tools

## The Command Center assistant

The dashboard embeds an on-prem **LLM chatbot** ("Command Center assistant").
`server.py` proxies chat to an **OpenAI-compatible** endpoint — **Ollama** on the
AI host by default. The proxy is fundamentally **read-only for telemetry** but
can invoke a small set of **guarded action tools**.

| Setting | Default | Meaning |
| --- | --- | --- |
| `LLM_ENABLED` | `1` | Enable the chat proxy |
| `LLM_BASE_URL` | `http://10.2.125.3:11434` | Ollama endpoint ([[02 - Network & Hosts\|AI host]]) |
| `LLM_MODEL` | `qwen3:30b-a3b-instruct-2507-q4_K_M` | Model id |
| `LLM_TEMPERATURE` | `0` | Deterministic — at 0.3, qwen3-30b intermittently skipped tool calls |
| `LLM_MAX_TOKENS` | `1024` | Reply cap |
| `LLM_TIMEOUT_SECONDS` | `120` | Completion timeout |
| `LLM_MAX_MESSAGES` | `24` | Cap on conversation the browser may send |
| `LLM_MAX_MESSAGE_CHARS` | `8000` | Per-message char cap |
| `LLM_INCLUDE_ROS_GRAPH` | `1` | Inject the ROS 2 node/topic graph into context |
| `LLM_API_KEY` | empty | Optional bearer for the LLM endpoint |

`/api/chat` runs a **tool loop** (`_chat_tool_loop`): each round forwards the
conversation + tool specs, dispatches requested calls locally, appends JSON
results as `tool` messages, and after `LLM_MAX_TOOL_ROUNDS` calls the model once
more **without** tools to force a final answer.

> [!note] Textual tool-call fallback
> Some models (qwen3-30b on short Turkish imperatives) emit a tool call as
> plain-text JSON instead of a structured `tool_calls` entry.
> `extract_textual_tool_call` promotes such embedded JSON to a real tool call so
> the **guarded handler still runs** — the guards are not bypassed.

## The tools

Built by `_chat_tool(...)` and assembled in `chat_tool_specs()`. Every handler
runs **locally on the robot PC** (in-process state or the `ros2` CLI); the only
network traffic is the chat completion itself.

| Tool | Kind | Purpose |
| --- | --- | --- |
| `get_joint_details` | read | Full live state of one motor/hand joint by name |
| `get_loco_status` | read | Locomotion state, motion mode, command history |
| `ros2_node_list` | read | List live ROS 2 nodes |
| `ros2_topic_list` | read | List live topics + message types |
| `ros2_node_info` | read | Pubs/subs/services of one node (name-validated) |
| `ros2_topic_info` | read | Type + pub/sub counts of one topic (name-validated) |
| `ros2_topic_echo` | read | Capture ONE message from a topic (name-validated, times out) |
| `chill_motors` | **GUARDED ACTION** | Damp all motors (robot goes limp) — the dashboard "Release" |
| `propose_arm_pose` | **STAGES A PREVIEW** | Propose joint angles → green digital-twin preview, awaiting approval (never moves on its own) — see [[20 - LLM Arm Pose Proposals & Mimic]] |
| `move` | **GUARDED ACTION** | Move arms to a saved named position (or an approved proposed pose) via the validated arm replay |

- `ros2_*` inputs pass `valid_ros2_name()` validation (rejects invalid names).
- `chill_motors` requires `confirm=true`; only fire on an explicit
  release/chill/damp/relax request.
- `move` requires `confirm=true` and a `position` from the **enum of saved
  named positions** (built live from `named_positions()`); it drives the same
  validated arm_sdk replay as the dashboard Move button (arms scope, closed-loop,
  safety-checked — see [[03 - Safety Interlocks]]).

### Tool feature flags

| Flag | Default | Effect |
| --- | --- | --- |
| `LLM_TOOLS_ENABLED` | `1` | Enable tool calling at all |
| `LLM_TOOL_CHILL_ENABLED` | `1` | Include `chill_motors` (dropped from specs if `0`) |
| `LLM_TOOL_MOVE_ENABLED` | `1` | Include `move` (only if saved positions exist) |
| `LLM_MAX_TOOL_ROUNDS` | `4` | Max tool rounds per chat turn |
| `LLM_MAX_TOOL_CALLS_PER_ROUND` | `5` | Max calls dispatched per round |
| `LLM_TOOL_OUTPUT_CHARS` | `6000` | Truncate each tool result |
| `ROS2_TOOL_TIMEOUT` | `6` | Timeout for `ros2` CLI tools |

> [!note] `track_person` tool (live, dark by default)
> The [[06 - Person Tracking (CV Feature)|tracking]] tool `track_person`
> (`action: start|stop`, `confirm: true`) is now implemented in `server.py`
> (`track_tool_spec` + `_tool_track`). It is double-gated: it only appears when
> **both** `LLM_TOOL_TRACK_ENABLED` and `TRACKING_ENABLED` are on, and it refuses
> unless `confirm=true`. MCP exposure is automatic. See
> [[06 - Person Tracking (CV Feature)#Implementation status]].

## The MCP endpoint (`POST /mcp`)

`/mcp` exposes the **same** chat tools — same specs, same `run_chat_tool`
dispatch, therefore **exactly the same guards** (chill confirm gate, ros2 name
validation, `LLM_TOOL_*` flags). This is the stable, client-agnostic LLM
interface; future motion skills should be added here as guarded tools.

| Setting | Default | Meaning |
| --- | --- | --- |
| `MCP_ENABLED` | `0` | Serve `POST /mcp` — **off by default** so autoupdate never silently opens it |
| `MCP_TOKEN` | empty | If set, requests must send `Authorization: Bearer <token>` |

- Protocol: stateless streamable HTTP, JSON responses only (no SSE, no
  sessions, no resumability). Notifications → `202`; `GET /mcp` → `405`.
- Supported methods: `initialize`, `ping`, `tools/list`, `tools/call`.
- Protocol versions: `2025-06-18`, `2025-03-26`, `2024-11-05`.
- `mcp_tool_descriptors()` converts the chat specs to MCP tool descriptors, so
  MCP exposure of any new tool is **automatic**.

Connect from Claude Code:

```bash
claude mcp add --transport http robot http://10.2.100.142:8088/mcp \
  --header "Authorization: Bearer <token>"
```

Or drive it with the on-prem LLM via the bundled stdlib client
`tools/mcp_agent.py` (discovers tools via `tools/list`, offers OpenAI-style
function specs, executes via `tools/call`):

```bash
python3 tools/mcp_agent.py --token <token> \
  -p "What is the robot's locomotion status right now?"
```

## Claude bridge backend (optional, vision-capable)

Besides the default on-prem qwen, `/api/chat` accepts a `backend` of `claude`,
which proxies to a **Claude Code bridge** (`tools/claude_bridge.py`) running on
the operator's machine — same context, same tools, same guards, but a
vision-capable model (Opus). A toggle in the chat header selects it.

| Setting | Default | Meaning |
| --- | --- | --- |
| `CLAUDE_BRIDGE_URL` | empty | Bridge base URL (e.g. `http://10.2.100.81:8399`); empty = backend unavailable |
| `CLAUDE_BRIDGE_MODEL` | `claude` | Model id passed to the bridge |
| `CLAUDE_BRIDGE_TOKEN` | empty | Optional bearer for the bridge |

The bridge is **required for the photo→pose [[20 - LLM Arm Pose Proposals & Mimic|mimic]] feature**:
the on-prem qwen is text-only, so any request carrying an image (`mimic_image`,
or the twin render when `LLM_TWIN_VISION_ENABLED`) is routed to the bridge. The
bridge delivers images to the CLI via `--input-format stream-json` — plain-text
stdin would drop them. Run it in the operator's own terminal so it persists.

## Voice (optional, off by default)

| Flag | Default | Endpoint |
| --- | --- | --- |
| `LLM_STT_ENABLED` | `0` | `/api/stt` → `LLM_STT_BASE_URL` (`:8001`, faster-whisper) |
| `LLM_TTS_ENABLED` | `0` | `/api/tts` → `LLM_TTS_BASE_URL` (`:8002`) |

`MAX_AUDIO_BYTES` (~15 MB), `MAX_TTS_TEXT_CHARS` (2000), `VOICE_TIMEOUT_SECONDS`
(60). TLS (`TLS_CERT`/`TLS_KEY`) is needed for the browser mic over a LAN IP
(secure-context rule).

## Related

[[04 - HTTP API Reference]] · [[03 - Safety Interlocks]] · [[06 - Person Tracking (CV Feature)]] · [[10 - Testing]]
</content>
