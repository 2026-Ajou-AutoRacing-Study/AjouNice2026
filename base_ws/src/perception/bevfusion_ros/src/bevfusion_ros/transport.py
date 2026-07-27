"""Reconnectable localhost TCP transport isolated for a future UDS pivot."""

import socket
import threading

from .protocol import (
    TYPE_ERROR,
    TYPE_REQUEST,
    TYPE_RESPONSE,
    ProtocolError,
    recv_message,
    send_message,
)


class TcpTransport:
    def __init__(self, host, port, connect_timeout, request_timeout):
        self.host = host
        self.port = int(port)
        self.connect_timeout = float(connect_timeout)
        self.request_timeout = float(request_timeout)
        self._socket = None
        self._lock = threading.Lock()

    def request(self, metadata, blobs):
        with self._lock:
            try:
                connection = self._connect()
                send_message(connection, TYPE_REQUEST, metadata, blobs)
                message_type, response, response_blobs = recv_message(connection)
                if message_type == TYPE_ERROR:
                    raise RuntimeError(response.get("error", "BEVFusion server error"))
                if message_type != TYPE_RESPONSE:
                    raise ProtocolError(
                        "unexpected response type {}".format(message_type)
                    )
                return response, response_blobs
            except Exception:
                self.close()
                raise

    def _connect(self):
        if self._socket is None:
            connection = socket.create_connection(
                (self.host, self.port), self.connect_timeout
            )
            connection.settimeout(self.request_timeout)
            self._socket = connection
        return self._socket

    def close(self):
        if self._socket is not None:
            try:
                self._socket.close()
            finally:
                self._socket = None
