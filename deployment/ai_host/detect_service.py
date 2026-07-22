"""Minimal person-detection + tracking HTTP service.

POST /detect?feed=<name> with a JPEG body ->
    JSON {persons: [{id,x1,y1,x2,y2,conf,cx,cy}], ms, w, h}
GET  /health -> {ok: true, feeds: [...]}

Coordinates are normalized 0..1 relative to image width/height. Each feed
keeps its own ByteTrack state (one YOLO instance per feed), so `id` is a
persistent track id for the same person across frames — it survives fast
motion (Kalman motion prediction) and brief occlusions (track buffer).
`id` is null for the first frame or two while a track is being confirmed.
"""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

import cv2
import numpy as np
from ultralytics import YOLO

PORT = 8188

_feeds: dict = {}
_feeds_guard = threading.Lock()


def feed_model(name):
    """One (model, lock) pair per feed; model.track(persist=True) is stateful."""
    with _feeds_guard:
        if name not in _feeds:
            _feeds[name] = (YOLO("yolov8n.pt"), threading.Lock())
        return _feeds[name]


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if urlsplit(self.path).path == "/health":
            with _feeds_guard:
                feeds = sorted(_feeds)
            self._send(200, {"ok": True, "feeds": feeds})
        else:
            self._send(404, {"error": "not found"})

    def do_POST(self):
        url = urlsplit(self.path)
        if url.path != "/detect":
            self._send(404, {"error": "not found"})
            return
        feed = (parse_qs(url.query).get("feed") or ["default"])[0]
        length = int(self.headers.get("Content-Length", 0))
        data = self.rfile.read(length)
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            self._send(400, {"error": "bad image"})
            return
        h, w = img.shape[:2]
        model, lock = feed_model(feed)
        t0 = time.time()
        with lock:
            results = model.track(
                img, classes=[0], conf=0.4, persist=True,
                tracker="bytetrack.yaml", verbose=False,
            )
        ms = (time.time() - t0) * 1000
        persons = []
        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            persons.append({
                "id": int(box.id[0]) if box.id is not None else None,
                "x1": x1 / w, "y1": y1 / h, "x2": x2 / w, "y2": y2 / h,
                "cx": (x1 + x2) / 2 / w, "cy": (y1 + y2) / 2 / h,
                "conf": float(box.conf[0]),
            })
        persons.sort(key=lambda p: (p["x2"] - p["x1"]) * (p["y2"] - p["y1"]), reverse=True)
        self._send(200, {"persons": persons, "ms": round(ms, 1), "w": w, "h": h, "feed": feed})

    def log_message(self, *args):
        pass


if __name__ == "__main__":
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
