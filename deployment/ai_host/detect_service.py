"""Minimal person-detection + tracking HTTP service.

POST /detect?feed=<name> with a JPEG body ->
    JSON {persons: [{id,x1,y1,x2,y2,conf,cx,cy}], ms, w, h}
GET  /health -> {ok: true, feeds: [...], model: ..., device: ...}

Coordinates are normalized 0..1 relative to image width/height. Each feed
keeps its own ByteTrack state (one YOLO instance per feed), so `id` is a
persistent track id for the same person across frames — it survives fast
motion (Kalman motion prediction) and brief occlusions (track buffer).
`id` is null for the first frame or two while a track is being confirmed.

Model selection: DETECT_MODEL env var (default yolo11m.pt — accurate and
~20 ms on the host GPU). Runs on CUDA when available. Each feed's model is
warmed up with a dummy frame at creation so the first real request is fast.
"""
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

import cv2
import numpy as np
import torch
from ultralytics import YOLO

PORT = 8188
MODEL_NAME = os.environ.get("DETECT_MODEL", "yolo11m-pose.engine")
DEVICE = 0 if torch.cuda.is_available() else "cpu"
CONF = float(os.environ.get("DETECT_CONF", "0.35"))
IMGSZ = int(os.environ.get("DETECT_IMGSZ", "640"))
_LOCAL_TRACKER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tracker.yaml")
TRACKER = os.environ.get(
    "DETECT_TRACKER",
    _LOCAL_TRACKER if os.path.exists(_LOCAL_TRACKER) else "bytetrack.yaml",
)

_feeds: dict = {}
_feeds_guard = threading.Lock()


def feed_model(name):
    """One (model, lock) pair per feed; model.track(persist=True) is stateful."""
    with _feeds_guard:
        if name not in _feeds:
            model = YOLO(MODEL_NAME)
            dummy = np.zeros((480, 640, 3), dtype=np.uint8)
            model.track(dummy, classes=[0], persist=True, verbose=False, device=DEVICE)
            _feeds[name] = (model, threading.Lock())
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
            self._send(200, {"ok": True, "feeds": feeds, "model": MODEL_NAME,
                             "device": str(DEVICE), "tracker": os.path.basename(TRACKER)})
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
                img, classes=[0], conf=CONF, imgsz=IMGSZ, persist=True,
                tracker=TRACKER, verbose=False, device=DEVICE,
            )
        ms = (time.time() - t0) * 1000
        # COCO keypoint indices used below: 0 nose, 3/4 ears, 5/6 shoulders,
        # 11/12 hips. Everything is normalized 0..1 like the boxes.
        kpts = getattr(results[0], "keypoints", None)
        kp_xy = kpts.xy.tolist() if kpts is not None and kpts.xy is not None else []
        kp_conf = kpts.conf.tolist() if kpts is not None and kpts.conf is not None else []
        persons = []
        for i, box in enumerate(results[0].boxes):
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            person = {
                "id": int(box.id[0]) if box.id is not None else None,
                "x1": x1 / w, "y1": y1 / h, "x2": x2 / w, "y2": y2 / h,
                "cx": (x1 + x2) / 2 / w, "cy": (y1 + y2) / 2 / h,
                "conf": float(box.conf[0]),
            }
            if i < len(kp_xy) and i < len(kp_conf):
                xy, kc = kp_xy[i], kp_conf[i]
                def kp(j):
                    if j < len(xy) and j < len(kc) and kc[j] >= 0.3:
                        return {"x": xy[j][0] / w, "y": xy[j][1] / h, "conf": round(kc[j], 3)}
                    return None
                named = {"nose": kp(0), "l_ear": kp(3), "r_ear": kp(4),
                         "l_shoulder": kp(5), "r_shoulder": kp(6),
                         "l_hip": kp(11), "r_hip": kp(12)}
                person["keypoints"] = {k: v for k, v in named.items() if v}
                # Head anchor: nose, else ear midpoint, else nothing (client
                # falls back to box top-center).
                if named["nose"]:
                    person["head"] = {"x": named["nose"]["x"], "y": named["nose"]["y"]}
                elif named["l_ear"] and named["r_ear"]:
                    person["head"] = {
                        "x": (named["l_ear"]["x"] + named["r_ear"]["x"]) / 2,
                        "y": (named["l_ear"]["y"] + named["r_ear"]["y"]) / 2,
                    }
            persons.append(person)
        persons.sort(key=lambda p: (p["x2"] - p["x1"]) * (p["y2"] - p["y1"]), reverse=True)
        self._send(200, {"persons": persons, "ms": round(ms, 1), "w": w, "h": h, "feed": feed})

    def log_message(self, *args):
        pass


if __name__ == "__main__":
    for known_feed in ("default", "webcam", "head"):
        feed_model(known_feed)  # warm known feeds so first requests are fast
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
