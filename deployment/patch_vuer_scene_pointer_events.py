#!/usr/bin/env python3
"""Patch Vuer scene meshes to publish pointer down/up events for XR controls."""

from __future__ import annotations

from pathlib import Path


CHUNKS_DIR = Path("/home/unitree/.micromamba/envs/tv/lib/python3.10/site-packages/vuer/client_build/assets/chunks")


def patch_chunk(path: Path) -> bool:
    text = path.read_text(encoding="utf-8", errors="ignore")
    if "ON_POINTER_DOWN" in text and "ON_POINTER_UP" in text:
        return False

    old = 'outlines:f,onClick:d,...h}){'
    new = 'outlines:f,onClick:d,onPointerDown:rtwPointerDown,onPointerUp:rtwPointerUp,...h}){'
    if old not in text:
        return False
    text = text.replace(old, new, 1)

    old_callback = 'w=y.useCallback(T=>{b.publish({ts:Date.now(),etype:"ON_CLICK",value:{key:e}}),d==null||d(T)},[d]);'
    new_callback = (
        'w=y.useCallback(T=>{b.publish({ts:Date.now(),etype:"ON_CLICK",value:{key:e}}),d==null||d(T)},[d]),'
        'rtwDown=y.useCallback(T=>{rtwPointerDown&&b.publish({ts:Date.now(),etype:"ON_POINTER_DOWN",value:{key:e}})},[rtwPointerDown]),'
        'rtwUp=y.useCallback(T=>{rtwPointerUp&&b.publish({ts:Date.now(),etype:"ON_POINTER_UP",value:{key:e}})},[rtwPointerUp]);'
    )
    if old_callback not in text:
        return False
    text = text.replace(old_callback, new_callback, 1)

    old_mesh = 'H.jsxs("mesh",{ref:m,onClick:w,...h,children:'
    new_mesh = 'H.jsxs("mesh",{ref:m,onClick:w,onPointerDown:rtwDown,onPointerUp:rtwUp,...h,children:'
    if old_mesh not in text:
        return False
    text = text.replace(old_mesh, new_mesh, 1)

    path.write_text(text, encoding="utf-8")
    return True


def main() -> int:
    if not CHUNKS_DIR.exists():
        return 0
    patched = 0
    for path in CHUNKS_DIR.glob("*.js"):
        if patch_chunk(path):
            patched += 1
    if patched == 0:
        marker_found = any("ON_POINTER_DOWN" in p.read_text(encoding="utf-8", errors="ignore") for p in CHUNKS_DIR.glob("*.js"))
        if not marker_found:
            raise SystemExit("Could not patch Vuer scene pointer events")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
