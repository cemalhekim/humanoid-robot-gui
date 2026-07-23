# Pose Feedback Loop (3b) — Implementation Plan (condensed)

**Goal:** Before the operator approves a staged pose, the chat shows a liked/disliked
card (independent of execution); every verdict/comment/execution lands in a CSV; the
liked/disliked examples feed back into qwen's prompt as learned anchors / avoid-list.
A chat-header icon button toggles the feedback UI.

Tasks (each: failing test → code → suite → commit):
1. Server feedback store: proposal meta memory (last 20 incl. request text), CSV
   append (`feedback/pose_feedback.csv`: timestamp_iso, proposal_id, event
   [liked|disliked|executed], request_text, joints_json, semantics_json, comment),
   `record_pose_feedback(payload)` validation; `move proposed` success appends an
   `executed` row.
2. Chat response carries `proposal: {id, targets}` when a propose succeeded in the
   turn; `store.chat` stashes the triggering user message for the meta record.
3. Learned examples: read CSV (last 8 liked / 4 disliked), append "LEARNED FROM
   OPERATOR FEEDBACK" section to the system prompt at chat time.
4. HTTP: `POST /api/pose/feedback` (both do_POST allowlists + dispatch).
5. UI: header toggle (localStorage `h1_pose_feedback`), feedback card with 👍/👎
   SVG buttons + optional comment input under replies that staged a proposal;
   cache-bust; node --check.
6. Local real-qwen smoke + CSV inspection; push to main; verify robot restart.
