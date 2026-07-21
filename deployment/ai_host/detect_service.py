"""Minimal person-detection HTTP service.

POST /detect with a JPEG body -> JSON {persons: [{x1,y1,x2,y2,conf,cx,cy}], ms}
GET  /health -> {ok: true}
Coordinates are normalized 0..1 relative to image width/height.
"""
import json
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
from ultralytics import YOLO

MODEL = YOLO("yolov8n.pt")
PORT = 8188


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/health":
            self._send(200, {"ok": True})
        else:
            self._send(404, {"error": "not found"})

    def do_POST(self):
        if self.path != "/detect":
            self._send(404, {"error": "not found"})
            return
        length = int(self.headers.get("Content-Length", 0))
        data = self.rfile.read(length)
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            self._send(400, {"error": "bad image"})
            return
        h, w = img.shape[:2]
        t0 = time.time()
        results = MODEL.predict(img, classes=[0], conf=0.4, verbose=False)
        ms = (time.time() - t0) * 1000
        persons = []
        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            persons.append({
                "x1": x1 / w, "y1": y1 / h, "x2": x2 / w, "y2": y2 / h,
                "cx": (x1 + x2) / 2 / w, "cy": (y1 + y2) / 2 / h,
                "conf": float(box.conf[0]),
            })
        persons.sort(key=lambda p: (p["x2"] - p["x1"]) * (p["y2"] - p["y1"]), reverse=True)
        self._send(200, {"persons": persons, "ms": round(ms, 1), "w": w, "h": h})

    def log_message(self, *args):
        pass


if __name__ == "__main__":
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
