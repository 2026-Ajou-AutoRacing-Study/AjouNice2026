#!/usr/bin/env python3
"""Check that the local BEVFusion TCP server accepts the versioned protocol."""

import argparse
import socket

from app.protocol import TYPE_PING, TYPE_PONG, recv_message, send_message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=29500)
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()
    with socket.create_connection((args.host, args.port), args.timeout) as connection:
        send_message(connection, TYPE_PING, {"client": "healthcheck"})
        message_type, metadata, _blobs = recv_message(connection)
    if message_type != TYPE_PONG or not metadata.get("ready"):
        raise SystemExit("BEVFusion server is not ready: {}".format(metadata))
    print(
        "BEVFusion server: ready; model_loaded={}".format(
            metadata.get("model_loaded", False)
        )
    )


if __name__ == "__main__":
    main()

