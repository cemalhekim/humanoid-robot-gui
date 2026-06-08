#!/usr/bin/env python3
"""Patch TeleImager client decoding for reliable XR rendering."""

from __future__ import annotations

from pathlib import Path


IMAGE_CLIENT = Path("/home/unitree/xr_teleoperate/teleop/teleimager/src/teleimager/image_client.py")

OLD = '''    def _decode_image(self, jpg_bytes):
        """Decode compressed image bytes to a BGR OpenCV frame."""
        if jpg_bytes is None:
            return None
        try:
            np_img = np.frombuffer(jpg_bytes, dtype=np.uint8)
            return cv2.imdecode(np_img, cv2.IMREAD_COLOR)
        except Exception as e:
            logger_mp.warning(f"[ZMQ_SubscriberThread] Failed to decode image: {e}")
            return None
'''

NEW = '''    def _decode_image(self, jpg_bytes):
        """Decode JPEG bytes to BGR numpy image."""
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


def main() -> int:
    if not IMAGE_CLIENT.exists():
        return 0
    text = IMAGE_CLIENT.read_text(encoding="utf-8")
    if NEW in text:
        return 0
    if OLD not in text:
        raise SystemExit("Could not find TeleImager PIL decoder block")
    IMAGE_CLIENT.write_text(text.replace(OLD, NEW), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
