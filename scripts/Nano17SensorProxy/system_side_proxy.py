import json
import socket
import sys
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class Client(Node):
    SERVER_IP = "192.168.1.6"
    SERVER_PORT = 9999

    LOCAL_IP = "192.168.1.5"
    LOCAL_PORT = 10000

    RECV_SIZE = 4096
    RECONNECT_DELAY = 1.0

    def __init__(self) -> None:
        super().__init__("nano_17_proxy")

        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            "~/ft",
            10,
        )

        self.socket: socket.socket | None = None
        self.receive_buffer = ""

    def create_socket(self) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Helps when restarting the client quickly.
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Optional: keep the TCP connection alive.
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)

        # Keep this only if the client must use this exact local port.
        sock.bind((self.LOCAL_IP, self.LOCAL_PORT))

        return sock

    def connect(self) -> None:
        while rclpy.ok():
            try:
                self.socket = self.create_socket()
                self.socket.connect((self.SERVER_IP, self.SERVER_PORT))

                self.receive_buffer = ""

                self.get_logger().info(
                    f"Connected to {self.SERVER_IP}:{self.SERVER_PORT}"
                )
                return

            except OSError as error:
                self.get_logger().warning(
                    f"Connection failed: {error}. Retrying..."
                )

                self.close_socket()
                time.sleep(self.RECONNECT_DELAY)

    @staticmethod
    def clean_line(line: str) -> str:
        """
        Remove characters that commonly appear around a JSON message.

        This does not attempt to repair corrupted JSON because doing so could
        silently publish incorrect force/torque values.
        """
        return line.strip().strip("\x00")

    @staticmethod
    def validate_data(data: Any) -> list[float] | None:
        """Ensure the decoded JSON is a numeric array."""
        if not isinstance(data, list):
            return None

        cleaned_data: list[float] = []

        for value in data:
            # bool is a subclass of int, so reject it explicitly.
            if isinstance(value, bool):
                return None

            try:
                cleaned_data.append(float(value))
            except (TypeError, ValueError, OverflowError):
                return None

        return cleaned_data

    def process_line(self, line: str) -> None:
        line = self.clean_line(line)

        if not line:
            return

        try:
            decoded_data = json.loads(line)
        except json.JSONDecodeError as error:
            self.get_logger().warning(
                f"Discarding invalid JSON: {error}. Data: {line!r}"
            )
            return

        data = self.validate_data(decoded_data)

        if data is None:
            self.get_logger().warning(
                f"Discarding invalid data array: {decoded_data!r}"
            )
            return

        msg = Float64MultiArray()
        msg.data = data
        self.publisher_.publish(msg)

    def receive(self) -> None:
        while rclpy.ok():
            if self.socket is None:
                self.connect()

            try:
                assert self.socket is not None

                raw_data = self.socket.recv(self.RECV_SIZE)

                # recv() returning b"" means the server closed the connection.
                if not raw_data:
                    raise ConnectionError("Server closed the connection")

                # Invalid bytes are discarded rather than crashing the node.
                self.receive_buffer += raw_data.decode(
                    "utf-8",
                    errors="ignore",
                )

                # The server must terminate every JSON message with "\n".
                while "\n" in self.receive_buffer:
                    line, self.receive_buffer = self.receive_buffer.split(
                        "\n",
                        1,
                    )
                    self.process_line(line)

            except (
                ConnectionError,
                ConnectionResetError,
                ConnectionAbortedError,
                BrokenPipeError,
                OSError,
            ) as error:
                self.get_logger().warning(
                    f"Socket connection lost: {error}"
                )
                self.close_socket()

                if rclpy.ok():
                    time.sleep(self.RECONNECT_DELAY)

    def close_socket(self) -> None:
        if self.socket is not None:
            try:
                self.socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

            self.socket.close()
            self.socket = None

    def destroy_node(self) -> bool:
        self.close_socket()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    client = Client()

    try:
        client.connect()
        client.receive()

    except KeyboardInterrupt:
        client.get_logger().info("Interrupted; exiting")

    finally:
        client.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()