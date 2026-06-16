#!/usr/bin/env python3
"""Add scoped pointer down/up events for the XR loco pad.

Vuer 0.0.60 publishes Box clicks as ON_CLICK, but it does not expose a press
start/release event to Python. This patch keeps the Vuer schema untouched and
only publishes extra events for scene objects whose key starts with rtw-loco-.
"""

from __future__ import annotations

from pathlib import Path


CHUNKS_DIR = Path(
    "/home/unitree/.micromamba/envs/tv/lib/python3.10/site-packages/"
    "vuer/client_build/assets/chunks"
)

OLD_SIGNATURE = "onClick:d,onPointerDown:rtwPointerDown,onPointerUp:rtwPointerUp,...h})"
NEW_SIGNATURE = "onClick:d,...h})"
OLD_CALLBACKS = (
    'w=y.useCallback(T=>{b.publish({ts:Date.now(),etype:"ON_CLICK",value:{key:e}}),'
    'd==null||d(T)},[d]),rtwDown=y.useCallback(T=>{rtwPointerDown&&b.publish'
    '({ts:Date.now(),etype:"ON_POINTER_DOWN",value:{key:e}})},[rtwPointerDown]),'
    'rtwUp=y.useCallback(T=>{rtwPointerUp&&b.publish({ts:Date.now(),'
    'etype:"ON_POINTER_UP",value:{key:e}})},[rtwPointerUp]);'
)
CLEAN_CALLBACKS = (
    'w=y.useCallback(T=>{b.publish({ts:Date.now(),etype:"ON_CLICK",value:{key:e}}),'
    'd==null||d(T)},[d]);'
)
OLD_MESH = 'H.jsxs("mesh",{ref:m,onClick:w,onPointerDown:rtwDown,onPointerUp:rtwUp,...h,children:'
CLEAN_MESH = 'H.jsxs("mesh",{ref:m,onClick:w,...h,children:'

TARGET_CALLBACKS = CLEAN_CALLBACKS
PATCHED_CALLBACKS = (
    'w=y.useCallback(T=>{b.publish({ts:Date.now(),etype:"ON_CLICK",value:{key:e}}),'
    'd==null||d(T)},[d]),rtwDown=y.useCallback(T=>{String(e).startsWith("rtw-loco-")'
    '&&b.publish({ts:Date.now(),etype:"RTW_LOCO_POINTER_DOWN",value:{key:e}})},[e,b]),'
    'rtwUp=y.useCallback(T=>{String(e).startsWith("rtw-loco-")&&b.publish'
    '({ts:Date.now(),etype:"RTW_LOCO_POINTER_UP",value:{key:e}})},[e,b]),'
    'rtwHover=y.useCallback(T=>{String(e).startsWith("rtw-loco-")&&'
    '(typeof window!="undefined"&&(window.__rtwLocoHoverKey=e),b.publish'
    '({ts:Date.now(),etype:"RTW_LOCO_HOVER",value:{key:e}}))},[e,b]),'
    'rtwLeave=y.useCallback(T=>{String(e).startsWith("rtw-loco-")&&'
    '(typeof window!="undefined"&&window.__rtwLocoHoverKey===e&&'
    '(window.__rtwLocoHoverKey=null),b.publish({ts:Date.now(),etype:'
    '"RTW_LOCO_HOVER",value:{key:null}}))},[e,b]);y.useEffect(()=>{if'
    '(typeof window=="undefined")return;window.__rtwLocoPublish=T=>b.publish(T);'
    'if(!window.__rtwLocoXRHoldPatched){window.__rtwLocoXRHoldPatched=!0;'
    'const T=()=>{const E=window.__rtwLocoHoverKey;E&&window.__rtwLocoPublish&&'
    'window.__rtwLocoPublish({ts:Date.now(),etype:"RTW_LOCO_POINTER_DOWN",'
    'value:{key:E}})},E=()=>{window.__rtwLocoPublish&&window.__rtwLocoPublish'
    '({ts:Date.now(),etype:"RTW_LOCO_POINTER_UP",value:{key:window.__rtwLocoHoverKey'
    '||null}})};window.addEventListener("pointerdown",T,!0);window.addEventListener'
    '("pointerup",E,!0);window.addEventListener("blur",E,!0);const N=navigator.xr;'
    'if(N&&N.requestSession&&!N.__rtwLocoWrapped){const R=N.requestSession.bind(N);'
    'N.__rtwLocoWrapped=!0;N.requestSession=(...O)=>R(...O).then(S=>{try{'
    'S.addEventListener("selectstart",T);S.addEventListener("selectend",E);'
    'S.addEventListener("squeezestart",T);S.addEventListener("squeezeend",E)}'
    'catch(A){}return S})}}},[b]);'
)
OLD_XR_HOLD_CALLBACKS = (
    'w=y.useCallback(T=>{b.publish({ts:Date.now(),etype:"ON_CLICK",value:{key:e}}),'
    'd==null||d(T)},[d]),rtwDown=y.useCallback(T=>{String(e).startsWith("rtw-loco-")'
    '&&b.publish({ts:Date.now(),etype:"RTW_LOCO_POINTER_DOWN",value:{key:e}})},[e,b]),'
    'rtwUp=y.useCallback(T=>{String(e).startsWith("rtw-loco-")&&b.publish'
    '({ts:Date.now(),etype:"RTW_LOCO_POINTER_UP",value:{key:e}})},[e,b]),'
    'rtwHover=y.useCallback(T=>{String(e).startsWith("rtw-loco-")&&b.publish'
    '({ts:Date.now(),etype:"RTW_LOCO_HOVER",value:{key:e}})},[e,b]),'
    'rtwLeave=y.useCallback(T=>{String(e).startsWith("rtw-loco-")&&b.publish'
    '({ts:Date.now(),etype:"RTW_LOCO_HOVER",value:{key:null}})},[e,b]);'
)
OLD_SCOPED_CALLBACKS = (
    'w=y.useCallback(T=>{b.publish({ts:Date.now(),etype:"ON_CLICK",value:{key:e}}),'
    'd==null||d(T)},[d]),rtwDown=y.useCallback(T=>{String(e).startsWith("rtw-loco-")'
    '&&b.publish({ts:Date.now(),etype:"RTW_LOCO_POINTER_DOWN",value:{key:e}})},[e,b]),'
    'rtwUp=y.useCallback(T=>{String(e).startsWith("rtw-loco-")&&b.publish'
    '({ts:Date.now(),etype:"RTW_LOCO_POINTER_UP",value:{key:e}})},[e,b]);'
)
TARGET_MESH = CLEAN_MESH
PATCHED_MESH = (
    'H.jsxs("mesh",{ref:m,onClick:w,onPointerDown:rtwDown,onPointerUp:rtwUp,'
    'onPointerCancel:rtwUp,onPointerEnter:rtwHover,onPointerOver:rtwHover,'
    'onPointerLeave:rtwLeave,onPointerOut:rtwLeave,...h,children:'
)
OLD_SCOPED_MESH = (
    'H.jsxs("mesh",{ref:m,onClick:w,onPointerDown:rtwDown,onPointerUp:rtwUp,'
    'onPointerCancel:rtwUp,onPointerOut:rtwUp,...h,children:'
)


def patch_text(text: str) -> tuple[str, bool]:
    """Return patched text and whether the input changed."""
    original = text

    # Remove the earlier broad pointer schema patch if it is still installed.
    text = text.replace(OLD_SIGNATURE, NEW_SIGNATURE)
    text = text.replace(OLD_CALLBACKS, CLEAN_CALLBACKS)
    text = text.replace(OLD_MESH, CLEAN_MESH)

    text = text.replace(OLD_XR_HOLD_CALLBACKS, PATCHED_CALLBACKS)
    text = text.replace(OLD_SCOPED_CALLBACKS, PATCHED_CALLBACKS)
    text = text.replace(OLD_SCOPED_MESH, PATCHED_MESH)

    if "RTW_LOCO_POINTER_DOWN" not in text:
        text = text.replace(TARGET_CALLBACKS, PATCHED_CALLBACKS)
        text = text.replace(TARGET_MESH, PATCHED_MESH)

    return text, text != original


def main() -> int:
    if not CHUNKS_DIR.exists():
        return 0

    changed = []
    found = False
    for path in CHUNKS_DIR.glob("*.js"):
        text = path.read_text(encoding="utf-8")
        if CLEAN_CALLBACKS not in text and "RTW_LOCO_POINTER_DOWN" not in text:
            continue
        found = True
        patched, did_change = patch_text(text)
        if did_change:
            path.write_text(patched, encoding="utf-8")
            changed.append(path)

    if not found:
        raise SystemExit("Could not find Vuer Box click handler in client bundle")

    leftovers = []
    for path in CHUNKS_DIR.glob("*.js"):
        text = path.read_text(encoding="utf-8")
        if "ON_POINTER_DOWN" in text or "ON_POINTER_UP" in text:
            leftovers.append(path)
    if leftovers:
        raise SystemExit(f"Old broad pointer patch still present: {leftovers}")

    print(f"Patched Vuer loco pointer events in {len(changed)} chunk(s).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
