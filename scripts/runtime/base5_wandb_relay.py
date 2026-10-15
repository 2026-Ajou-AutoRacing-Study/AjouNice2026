#!/usr/bin/env python3
"""Loopback-only HTTPS CONNECT relay for W&B/Drive, with TLS end-to-end.

No credentials, HTTP request bodies or host URLs are logged. Only W&B and
designated Google Drive/storage endpoints are allowed; not an open web proxy.
"""
import http.server
import select
import socket


def allowed(host, port):
    host = host.lower().rstrip('.')
    return port == 443 and (host == 'wandb.ai' or host.endswith('.wandb.ai') or
                           host == 'wandb.com' or host.endswith('.wandb.com') or
                           host in ('storage.googleapis.com', 'www.googleapis.com',
                                    'content.googleapis.com', 'drive.googleapis.com'))


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_CONNECT(self):
        try:
            host, raw_port = self.path.rsplit(':', 1)
            port = int(raw_port)
            if not allowed(host, port):
                self.send_error(403)
                return
            upstream = socket.create_connection((host, port), timeout=10)
        except (ValueError, OSError):
            self.send_error(502)
            return
        self.send_response(200, 'Connection Established')
        self.end_headers()
        self.connection.settimeout(15)
        upstream.settimeout(15)
        try:
            while True:
                ready, _, _ = select.select([self.connection, upstream], [], [], 90)
                if not ready:
                    break
                for source in ready:
                    payload = source.recv(65536)
                    if not payload:
                        return
                    target = upstream if source is self.connection else self.connection
                    target.sendall(payload)
        except OSError:
            pass
        finally:
            upstream.close()


def server(port):
    result = http.server.ThreadingHTTPServer(('127.0.0.1', port), Handler)
    result.daemon_threads = True
    return result
