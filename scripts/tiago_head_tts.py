#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import socket
import threading

from tts_msgs.action import TTS  # Replace with your actual action interface

# Laptop address (receiver of the "finished" message)
LAPTOP_IP = '10.68.0.131'
LAPTOP_PORT = 9092  # Use any open port you'll listen on

# Port this machine (Robot B) listens on
HEAD_LISTEN_PORT = 9091


class TTSClient(Node):
    def __init__(self):
        super().__init__('tts_action_client')
        self._client = ActionClient(self, TTS, '/tts_engine/tts')

        self.get_logger().info('Waiting for TTS action server...')
        self._client.wait_for_server()
        self.get_logger().info('TTS action server available.')

        # Start the TCP listener
        self.socket_thread = threading.Thread(target=self.listen_for_messages, daemon=True)
        self.socket_thread.start()

    def listen_for_messages(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.bind(('', HEAD_LISTEN_PORT))
            server_socket.listen(5)
            self.get_logger().info(f'Listening for TCP messages on port {HEAD_LISTEN_PORT}...')

            while True:
                conn, addr = server_socket.accept()
                with conn:
                    try:
                        data = conn.recv(1024).decode('utf-8').strip()
                        self.get_logger().info(f'Received: "{data}" from {addr}')
                        if data:
                            tts_input = self.lookup_text(data)
                            if tts_input:
                                self.send_tts(tts_input)
                                conn.sendall(b"ok")
                            else:
                                self.get_logger().warn(f'Unrecognised message: "{data}"')
                                conn.sendall(b"invalid")
                    except Exception as e:
                        self.get_logger().error(f'Error while handling message from {addr}: {e}')

    def lookup_text(self, message):
        phrases = {
            "high_explicit_correct_1_1": "Mind assisting during the transfer? If any of it spills, we’ll be under the required dose.",
            "high_explicit_correct_1_2": "Yes.",
            "high_explicit_correct_2_1": "That won’t mix itself.",
            "high_explicit_correct_2_2": "Yes.",
            "high_explicit_correct_3_1": "Another one, please.",
            "high_explicit_correct_3_2": "Yes."
        }
        return phrases.get(message)

    def send_tts(self, text):
        goal_msg = TTS.Goal()
        goal_msg.input = text
        self.get_logger().info(f'Sending TTS goal: "{text}"')

        future = self._client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('TTS goal was rejected.')
            return

        self.get_logger().info('TTS goal accepted.')
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_msg:
            self.get_logger().error(f"TTS error: {result.error_msg}")
        else:
            self.get_logger().info("TTS action completed successfully.")

        self.send_message_to_robot_a("finished")

    def send_message_to_robot_a(self, message):
        try:
            with socket.create_connection((LAPTOP_IP, LAPTOP_PORT), timeout=5) as client_socket:
                client_socket.sendall(message.encode('utf-8'))
                self.get_logger().info(f'Sent message to Robot A: "{message}"')
        except Exception as e:
            self.get_logger().error(f'Failed to send message to Robot A: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TTSClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
