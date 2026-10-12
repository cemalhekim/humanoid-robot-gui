#!/usr/bin/env python3
"""Patch the TeleImager client to decode JPEG frames with PIL instead of cv2.

On the robot's `tv` env cv2.imdecode is unusable (numpy/OpenCV ABI mismatch:
even a 4-byte buffer fails with "buf is not a numpy array, neither a scalar"),
so every head-camera frame was dropped and the headset showed no video
(2026-10-12). The patch swaps the body of `_decode_image` for a PIL decoder and
adds the imports it needs. Matched by method name, not by the upstream body,
so a reworded docstring upstream cannot silently un-apply it.
"""

from __future__ import annotations

import sys
from pathlib import Path


IMAGE_CLIENT = Path("/home/unitree/xr_teleoperate/teleop/teleimager/src/teleimager/image_client.py")

METHOD_HEAD = "    def _decode_image(self, jpg_bytes):\n"

NEW = '''    def _decode_image(self, jpg_bytes):
        """Decode JPEG bytes to a BGR numpy image (PIL; cv2.imdecode is broken here)."""
        if jpg_bytes is None:
            return None
        try:
            with Image.open(BytesIO(jpg_bytes)) as img:
                rgb = np.asarray(img.convert("RGB"), dtype=np.uint8)
                return np.ascontiguousarray(rgb[:, :, ::-1])
        except Exception as e:
            logger_mp.warning(f"[ZMQ_SubscriberThread] Failed to decode image: {e}")
            return None

'''

IMPORTS = ("from io import BytesIO\n", "from PIL import Image\n")


def ensure_imports(text: str) -> str:
    anchor = "import numpy as np\n"
    if anchor not in text:
        raise SystemExit("image_client.py: 'import numpy as np' anchor not found")
    missing = "".join(line for line in IMPORTS if line not in text)
    return text.replace(anchor, anchor + missing, 1) if missing else text


def patch(text: str) -> str:
    text = ensure_imports(text)
    if NEW in text:
        return text
    start = text.find(METHOD_HEAD)
    if start < 0:
        raise SystemExit("image_client.py: _decode_image method not found")
    end = text.find("\n    def ", start + len(METHOD_HEAD))
    if end < 0:
        raise SystemExit("image_client.py: end of _decode_image not found")
    return text[:start] + NEW + text[end + 1:]


def main(argv: list[str]) -> int:
    path = Path(argv[1]) if len(argv) > 1 else IMAGE_CLIENT
    if not path.exists():
        return 0
    text = path.read_text(encoding="utf-8")
    patched = patch(text)
    if patched != text:
        path.write_text(patched, encoding="utf-8")
        print(f"patched {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
