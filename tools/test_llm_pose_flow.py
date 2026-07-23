"""Manual integration test: LLM pose-proposal flow against the real qwen.

Run with a local server up (python3 server.py). Requires AI-DEV reachable.
Verifies the full chain WITHOUT any robot: a natural-language request stages a
proposal, the snapshot exposes it for the green twin, and the operator's
"okay" produces a move attempt that the no-DDS gate blocks (ok=false, proposal
retained) — proving no motion path exists on a robot-less host.

Usage: python3 tools/test_llm_pose_flow.py [http://localhost:8088] [request]
"""
from __future__ import annotations

import json
import sys
import urllib.request

BASE = sys.argv[1] if len(sys.argv) > 1 else "http://localhost:8088"
REQUEST = sys.argv[2] if len(sys.argv) > 2 else "elini one dogru uzat"


def post_chat(messages):
    body = json.dumps({"messages": messages}).encode()
    req = urllib.request.Request(f"{BASE}/api/chat", data=body, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=240) as response:
        return json.load(response)


def get(path):
    with urllib.request.urlopen(f"{BASE}{path}", timeout=10) as response:
        return json.load(response)


def check(label, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {label}  {detail}")
    return ok


def main() -> int:
    failures = 0
    history = [{"role": "user", "content": REQUEST}]
    reply = post_chat(history)
    used = reply.get("tools_used") or []
    names = [t.get("name") for t in used]
    failures += not check("qwen called propose_arm_pose", "propose_arm_pose" in names, str(names))
    failures += not check("qwen did NOT move in step 1", "move" not in names, str(names))
    proposal = get("/api/state").get("arm_proposal")
    failures += not check("snapshot exposes proposal (green twin data)", bool(proposal),
                          json.dumps(proposal)[:160] if proposal else "absent")
    print(f"       qwen: {reply.get('reply', '')[:200]}")

    history += [
        {"role": "assistant", "content": reply.get("reply", ""), "tools_used": used},
        {"role": "user", "content": "okay"},
    ]
    reply2 = post_chat(history)
    used2 = reply2.get("tools_used") or []
    moves = [t for t in used2 if t.get("name") == "move"]
    failures += not check("okay triggers move", bool(moves), str([t.get("name") for t in used2]))
    args_ok = moves and moves[-1].get("arguments", {}).get("position") == "proposed"
    failures += not check("move targets the staged proposal", bool(args_ok), json.dumps(moves)[:160])
    # No DDS locally: execution MUST fail closed and keep the proposal for retry.
    failures += not check("no-DDS gate blocked execution (robot untouched)",
                          bool(moves) and not moves[-1].get("ok"), json.dumps(moves)[:160])
    retained = get("/api/state").get("arm_proposal")
    failures += not check("proposal retained after blocked execution", bool(retained))
    print(f"       qwen: {reply2.get('reply', '')[:200]}")
    print("ALL PASS" if not failures else f"{failures} FAILURES")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
