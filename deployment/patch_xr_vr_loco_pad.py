#!/usr/bin/env python3
"""Inject a Vision Pro floating loco pad into Vuer and proxy commands locally."""

from __future__ import annotations

from pathlib import Path


VUER_ROOT = Path("/home/unitree/.micromamba/envs/tv/lib/python3.10/site-packages/vuer")
VUER_SERVER = VUER_ROOT / "server.py"
VUER_INDEX = VUER_ROOT / "client_build/index.html"

SERVER_MARKER = "# robot_telemetry_web xr loco proxy"
INDEX_START = "<!-- robot_telemetry_web xr loco pad start -->"
INDEX_END = "<!-- robot_telemetry_web xr loco pad end -->"

SERVER_HELPER = f'''
{SERVER_MARKER}
import json as _rtw_json
import urllib.error as _rtw_urllib_error
import urllib.request as _rtw_urllib_request


async def _rtw_xr_loco_proxy(request):
    try:
        body = await request.read()
        payload = body or b"{{}}"

        def _post_loco():
            req = _rtw_urllib_request.Request(
                "http://127.0.0.1:8088/api/loco/command",
                data=payload,
                headers={{"Content-Type": "application/json"}},
                method="POST",
            )
            try:
                with _rtw_urllib_request.urlopen(req, timeout=2.0) as resp:
                    return resp.status, resp.read().decode("utf-8", "replace")
            except _rtw_urllib_error.HTTPError as exc:
                return exc.code, exc.read().decode("utf-8", "replace")

        status, text = await asyncio.to_thread(_post_loco)
        return Response(status=status, text=text, content_type="application/json")
    except Exception as exc:
        text = _rtw_json.dumps({{"ok": False, "error": str(exc)}})
        return Response(status=502, text=text, content_type="application/json")
'''

INDEX_INJECT = f'''{INDEX_START}
<style>
  #rtw-xr-loco-pad {{
    position: fixed;
    right: max(22px, env(safe-area-inset-right));
    bottom: max(26px, env(safe-area-inset-bottom));
    z-index: 2147483647;
    width: 210px;
    height: 210px;
    display: grid;
    grid-template-columns: repeat(3, 64px);
    grid-template-rows: repeat(3, 64px);
    gap: 9px;
    place-content: center;
    border-radius: 999px;
    background: rgba(7, 10, 14, 0.48);
    border: 1px solid rgba(255, 255, 255, 0.24);
    box-shadow: 0 18px 60px rgba(0, 0, 0, 0.34);
    -webkit-backdrop-filter: blur(18px);
    backdrop-filter: blur(18px);
    -webkit-user-select: none;
    user-select: none;
    -webkit-touch-callout: none;
    touch-action: none;
  }}
  #rtw-xr-loco-pad button {{
    width: 64px;
    height: 64px;
    border: 0;
    border-radius: 999px;
    display: grid;
    place-items: center;
    color: #fff;
    background: rgba(255, 255, 255, 0.20);
    box-shadow: inset 0 0 0 1px rgba(255, 255, 255, 0.18);
    font: 800 30px/1 system-ui, -apple-system, BlinkMacSystemFont, sans-serif;
    -webkit-user-select: none;
    user-select: none;
    -webkit-touch-callout: none;
    touch-action: none;
  }}
  #rtw-xr-loco-pad button[data-stop] {{
    background: rgba(255, 68, 68, 0.86);
    font-size: 27px;
  }}
  #rtw-xr-loco-pad button.is-held {{
    background: rgba(54, 130, 255, 0.90);
    transform: scale(0.94);
  }}
  #rtw-xr-loco-pad [data-preset="turn-right"] {{ grid-column: 1; grid-row: 1; }}
  #rtw-xr-loco-pad [data-preset="forward"] {{ grid-column: 2; grid-row: 1; }}
  #rtw-xr-loco-pad [data-preset="turn-left"] {{ grid-column: 3; grid-row: 1; }}
  #rtw-xr-loco-pad [data-preset="left"] {{ grid-column: 1; grid-row: 2; }}
  #rtw-xr-loco-pad [data-stop] {{ grid-column: 2; grid-row: 2; }}
  #rtw-xr-loco-pad [data-preset="right"] {{ grid-column: 3; grid-row: 2; }}
  #rtw-xr-loco-pad [data-preset="back"] {{ grid-column: 2; grid-row: 3; }}
  @media (max-width: 720px) {{
    #rtw-xr-loco-pad {{
      width: 180px;
      height: 180px;
      grid-template-columns: repeat(3, 54px);
      grid-template-rows: repeat(3, 54px);
      gap: 7px;
    }}
    #rtw-xr-loco-pad button {{
      width: 54px;
      height: 54px;
      font-size: 25px;
    }}
  }}
</style>
<script>
(() => {{
  if (window.__rtwXrLocoPad) return;
  window.__rtwXrLocoPad = true;

  const presets = {{
    forward: [0.5, 0, 0],
    back: [-0.5, 0, 0],
    left: [0, 0.5, 0],
    right: [0, -0.5, 0],
    "turn-left": [0, 0, 0.5],
    "turn-right": [0, 0, -0.5],
  }};

  let hold = null;

  function payload(action, values) {{
    const [vx, vy, vyaw] = values || [0, 0, 0];
    return {{
      action,
      armed: true,
      i_understand_risk: true,
      vx,
      vy,
      vyaw,
      duration: 1,
      continuous_move: true,
    }};
  }}

  function command(action, values) {{
    return fetch("/xr-loco/command", {{
      method: "POST",
      headers: {{ "Content-Type": "application/json" }},
      body: JSON.stringify(payload(action, values)),
      keepalive: true,
    }}).catch(() => {{}});
  }}

  function stop(send = true) {{
    if (!hold) return;
    window.clearInterval(hold.timer);
    hold.button.classList.remove("is-held");
    hold = null;
    if (send) command("stop_move");
  }}

  function start(event) {{
    const button = event.currentTarget;
    const preset = button.dataset.preset;
    if (!preset || event.button > 0) return;
    event.preventDefault();
    stop(false);
    button.setPointerCapture?.(event.pointerId);
    button.classList.add("is-held");
    const values = presets[preset];
    command("move", values);
    hold = {{
      button,
      timer: window.setInterval(() => command("move", values), 300),
    }};
  }}

  function addButton(pad, preset, label, icon) {{
    const button = document.createElement("button");
    button.type = "button";
    button.dataset.preset = preset;
    button.setAttribute("aria-label", label);
    button.innerHTML = icon;
    button.addEventListener("pointerdown", start);
    button.addEventListener("pointerup", () => stop(true));
    button.addEventListener("pointercancel", () => stop(true));
    button.addEventListener("lostpointercapture", () => stop(true));
    button.addEventListener("click", (event) => event.preventDefault());
    pad.appendChild(button);
  }}

  function mount() {{
    if (document.getElementById("rtw-xr-loco-pad")) return;
    const pad = document.createElement("div");
    pad.id = "rtw-xr-loco-pad";
    pad.setAttribute("aria-label", "XR loco controls");
    addButton(pad, "turn-right", "Turn right", "&#8634;");
    addButton(pad, "forward", "Forward", "&#9650;");
    addButton(pad, "turn-left", "Turn left", "&#8635;");
    addButton(pad, "left", "Left", "&#9664;");
    const stopButton = document.createElement("button");
    stopButton.type = "button";
    stopButton.dataset.stop = "1";
    stopButton.setAttribute("aria-label", "Stop");
    stopButton.textContent = "X";
    stopButton.addEventListener("click", () => {{
      stop(false);
      command("stop_move");
    }});
    pad.appendChild(stopButton);
    addButton(pad, "right", "Right", "&#9654;");
    addButton(pad, "back", "Back", "&#9660;");
    document.body.appendChild(pad);
  }}

  if (document.readyState === "loading") {{
    document.addEventListener("DOMContentLoaded", mount, {{ once: true }});
  }} else {{
    mount();
  }}
  window.addEventListener("blur", () => stop(true));
  document.addEventListener("visibilitychange", () => {{
    if (document.hidden) stop(true);
  }});
}})();
</script>
{INDEX_END}'''


def replace_between(text: str, start: str, end: str, replacement: str) -> str:
    start_index = text.find(start)
    if start_index == -1:
        return text
    end_index = text.find(end, start_index)
    if end_index == -1:
        raise SystemExit(f"Found {start} without {end}")
    return text[:start_index] + replacement + text[end_index + len(end) :]


def patch_server() -> None:
    if not VUER_SERVER.exists():
        return
    text = VUER_SERVER.read_text(encoding="utf-8")
    if SERVER_MARKER not in text:
        import_anchor = "from vuer.types import EventHandler, SocketHandler\n"
        if import_anchor not in text:
            raise SystemExit("Could not find Vuer import anchor")
        text = text.replace(import_anchor, import_anchor + SERVER_HELPER, 1)

    route = '        self._add_route("/xr-loco/command", _rtw_xr_loco_proxy, method="POST")\n'
    if route not in text:
        anchor = '        self._add_route("/relay", self.relay, method="POST")\n'
        if anchor not in text:
            raise SystemExit("Could not find Vuer route anchor")
        text = text.replace(anchor, route + anchor, 1)
    VUER_SERVER.write_text(text, encoding="utf-8")


def patch_index() -> None:
    if not VUER_INDEX.exists():
        return
    text = VUER_INDEX.read_text(encoding="utf-8")
    text = replace_between(text, INDEX_START, INDEX_END, INDEX_INJECT)
    if INDEX_START not in text:
        if "</body>" not in text:
            raise SystemExit("Could not find Vuer index body close")
        text = text.replace("</body>", INDEX_INJECT + "\n      </body>", 1)
    VUER_INDEX.write_text(text, encoding="utf-8")


def main() -> int:
    patch_server()
    patch_index()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
