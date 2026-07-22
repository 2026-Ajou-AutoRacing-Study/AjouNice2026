"""Verify persistent localhost TCP interoperability between ROS and Docker modules."""

import socket
import threading
import unittest

from app.protocol import TYPE_REQUEST, TYPE_RESPONSE, recv_message, send_message
from bevfusion_ros.transport import TcpTransport


class TcpIntegrationTest(unittest.TestCase):
    def test_ros_client_to_docker_protocol(self):
        ready = threading.Event()
        result = {}

        def server_worker():
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
                server.bind(("127.0.0.1", 0))
                server.listen(1)
                result["port"] = server.getsockname()[1]
                ready.set()
                connection, _address = server.accept()
                with connection:
                    message_type, metadata, blobs = recv_message(connection)
                    result["request"] = (message_type, metadata, blobs)
                    send_message(
                        connection,
                        TYPE_RESPONSE,
                        {"operation": "inference", "detections": []},
                    )

        thread = threading.Thread(target=server_worker)
        thread.start()
        self.assertTrue(ready.wait(2.0))
        client = TcpTransport("127.0.0.1", result["port"], 1.0, 2.0)
        try:
            response, response_blobs = client.request(
                {"operation": "inference", "sequence": 1}, {"points": b"1234"}
            )
        finally:
            client.close()
        thread.join(2.0)
        self.assertFalse(thread.is_alive())
        self.assertEqual(result["request"][0], TYPE_REQUEST)
        self.assertEqual(result["request"][2]["points"], b"1234")
        self.assertEqual(response["operation"], "inference")
        self.assertEqual(response_blobs, {})


if __name__ == "__main__":
    unittest.main()

