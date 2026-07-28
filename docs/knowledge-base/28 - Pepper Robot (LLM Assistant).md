# 28 - Pepper Robot (LLM Assistant)

A second robot on the lab network: a **SoftBank/Aldebaran Pepper** (not part of
the H1-2 dashboard stack, but same 10.2.100.x Wi-Fi segment). Surveyed
2026-07-28 over SSH.

## Host

| What | Value |
| --- | --- |
| IP / SSH | `nao@10.2.100.187` (password in the local secrets file, never in repo) |
| Hostname | `Pepper` |
| OS | Gentoo Linux, kernel 4.0.4-rt1-aldebaran-rt, 32-bit i686 (Atom E3845) |
| NAOqi | 2.5.5 (`naoqi-bin`, qicli at `/opt/aldebaran/bin/qicli`) |
| Python | System Python 2.7 (official NAOqi bindings); Python 3.7 via `/home/nao/miniconda3` (`/home/nao/bin/python3`) |
| Disk | 1.5 GB root at 75 % full; 25 GB `/data` mostly free |

## What runs on it: `pepper_llm`

`/home/nao/pepper_llm` — a self-contained German-language voice assistant
(≈680 lines of Python 3, no external deps, stdlib `urllib` only):

1. **Record** short mic chunks (default 2 s, 16 kHz mono) — done by a tiny
   Python 2 helper `py2_audio_record.py` using native `ALAudioRecorder`
   bindings, because the qicli bridge can't pass the channel vector.
2. **VAD** — plain RMS threshold (`vad_rms_threshold: 900`) in `vad.py`.
3. **STT** — OpenAI-compatible `/audio/transcriptions` (Whisper, `de`), with a
   hallucination filter for the classic Whisper artifacts ("Untertitel der
   Amara.org-Community", ZDF/funk credits).
4. **LLM** — OpenAI-compatible `/chat/completions` (default `gpt-4o-mini`,
   max 90 tokens, rolling 8-turn memory persisted to `memory/conversation.json`).
5. **Speak** — `ALAnimatedSpeech` (contextual body language).
6. **Tablet UI** — local `ThreadingHTTPServer` on port **8088** serving a
   deliberately JavaScript-free 1990s-style HTML page (Pepper's WebView is
   ancient and flickers otherwise); shown via `ALTabletService.showWebview`
   pointing at `http://198.18.0.1:8088` (robot-internal tablet network).
   The single button toggles listening via `GET /action?listening=0|1`;
   state lives in `web/state.json`.

Config: `pepper_llm/config.json` + `system_prompt.txt` (persona: friendly
robot, German, max two short sentences). Secrets: `pepper_llm/.env`
(`OPENAI_API_KEY`, `LLM_API_BASE`, `LLM_MODEL`, `STT_MODEL`) loaded by
`start.sh`. Tests exist under `pepper_llm/tests/` (unittest, mockable).

### Python 3 → NAOqi bridge

`/home/nao/pepper_py3/pepper_qicli.py`: wraps `/opt/aldebaran/bin/qicli`
subprocess calls in a small Python 3 API (`call()`, `get()`, `Pepper` class
with `say/animated_say/posture/wake_up/move_to`). Slower than native bindings
but the only way to use Python 3 on this image.

## How it starts

- **systemd**: `pepper-llm.service` (enabled, currently inactive) →
  `start.sh` → sources `.env` → `app.py`.
- **Choregraphe behavior**: `pepper-llm` package installed under
  `~/.local/share/PackageManager/apps/pepper-llm` (built by
  `~/install_pepper_llm_app.py`, manifest `robotRequirement model="JULIETTE"`)
  so AutonomousLife can launch/focus it from the tablet.
- AutonomousLife is kept in `solitary` (not disabled) and
  `stopFocus()` is called so dialog activities don't steal the tablet;
  a configurable watchdog (`tablet_keep_front_seconds`, currently off —
  repeated `showWebview` causes flicker/white screen) can re-front the page.

## State as of 2026-07-28

- NAOqi and stock behaviors run; **pepper_llm is not running**.
- Last log (`pepper_app.log`, 2026-07-14): `Can't find service:
  ALAudioRecorder` from the Python 2 recorder helper — the audio recorder
  service wasn't up when the app last started. Likely start-ordering issue
  (the `.new` service file adds `ExecStartPre=/bin/sleep 45` +
  `Restart=always` to address exactly this, but isn't installed yet).
- Home dir also holds ~20 one-shot `fix_*.py` / `patch_*.py` maintenance
  scripts (UTF-8/umlauts, tablet flicker, race conditions, listen timeout,
  sound-recognition tuning) — history of debugging, applied ad hoc; and
  timestamped `pepper_llm.bak-*` backups.

Related: [[02 - Network & Hosts]] (H1-2 hosts), [[05 - Chat & MCP Tools]]
(the H1-2 dashboard's own chatbot — separate implementation).
