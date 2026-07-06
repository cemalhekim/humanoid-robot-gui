#!/usr/bin/env python3
"""Read-only reverse proxy for remote dashboard viewing.

This intentionally blocks every non-GET/HEAD request so the public tunnel can
show telemetry, camera, static assets, and recordings without exposing robot
motion command endpoints.
"""

from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import urllib.error
import urllib.parse
import urllib.request


HOP_BY_HOP_HEADERS = {
    "connection",
    "keep-alive",
    "proxy-authenticate",
    "proxy-authorization",
    "te",
    "trailer",
    "transfer-encoding",
    "upgrade",
}


class ReadOnlyProxy(BaseHTTPRequestHandler):
    upstream: str

    def do_GET(self) -> None:
        self.proxy()

    def do_HEAD(self) -> None:
        self.proxy(head_only=True)

    def do_POST(self) -> None:
        self.block()

    def do_PUT(self) -> None:
        self.block()

    def do_PATCH(self) -> None:
        self.block()

    def do_DELETE(self) -> None:
        self.block()

    def block(self) -> None:
        self.send_response(403)
        self.send_header("content-type", "application/json; charset=utf-8")
        self.end_headers()
        self.wfile.write(b'{"ok":false,"error":"Remote tunnel is read-only."}\n')

    def proxy(self, head_only: bool = False) -> None:
        target = urllib.parse.urljoin(self.upstream, self.path)
        request = urllib.request.Request(
            target,
            method="HEAD" if head_only else "GET",
            headers={
                "accept": self.headers.get("accept", "*/*"),
                "user-agent": self.headers.get("user-agent", "read-only-proxy"),
            },
        )

        try:
            with urllib.request.urlopen(request, timeout=60) as response:
                self.send_response(response.status)
                for key, value in response.headers.items():
                    if key.lower() not in HOP_BY_HOP_HEADERS:
                        self.send_header(key, value)
                self.send_header("x-robot-dashboard-proxy", "read-only")
                self.end_headers()
                if head_only:
                    return
                while True:
                    chunk = response.read(64 * 1024)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    self.wfile.flush()
        except urllib.error.HTTPError as error:
            self.send_response(error.code)
            self.send_header("content-type", "text/plain; charset=utf-8")
            self.end_headers()
            if not head_only:
                self.wfile.write(error.read())
        except Exception as error:
            self.send_response(502)
            self.send_header("content-type", "text/plain; charset=utf-8")
            self.end_headers()
            if not head_only:
                self.wfile.write(f"Upstream unavailable: {error}\n".encode())

    def log_message(self, fmt: str, *args: object) -> None:
        print(f"{self.address_string()} - {fmt % args}", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="Read-only robot dashboard proxy")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18088)
    parser.add_argument("--upstream", default="http://127.0.0.1:8088")
    args = parser.parse_args()

    ReadOnlyProxy.upstream = args.upstream.rstrip("/") + "/"
    server = ThreadingHTTPServer((args.host, args.port), ReadOnlyProxy)
    print(
        f"Read-only proxy listening on http://{args.host}:{args.port} -> {ReadOnlyProxy.upstream}",
        flush=True,
    )
    server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
