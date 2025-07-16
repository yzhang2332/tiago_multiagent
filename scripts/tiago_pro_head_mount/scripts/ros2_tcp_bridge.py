import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from tts_msgs.action import TTS
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from utils.tcp import TCPClient, TCPServer

LAPTOP_IP = "yanzhang-ASUS.local"
LAPTOP_PORT = 9092
HEAD_LISTEN_PORT = 9091

client = TCPClient(LAPTOP_IP, LAPTOP_PORT)

class TTSBridge(Node):
    def __init__(self):
        super().__init__('tts_bridge')
        self.client = ActionClient(self, TTS, '/tts_engine/tts')
        self.client.wait_for_server()
        self.get_logger().info("TTS server ready.")

        self.speech_pub = self.create_publisher(String, '/communication_hub/robot_speech', 10)

        self.server = TCPServer(port=HEAD_LISTEN_PORT, handler=self.handle_incoming_message)
        self.server.start()

    def handle_incoming_message(self, message, addr):
        text = self.lookup_text(message)
        if not text:
            return "invalid"

        # # Publish to /communication_hub/robot_speech
        # msg = String()
        # msg.data = text

        # Delay speech_pub by 0.5 seconds
        self.timer = self.create_timer(0.5, lambda: self.publish_speech_once(text))

        # Send to TTS engine
        goal = TTS.Goal()
        goal.input = text
        future = self.client.send_goal_async(goal)

        future.add_done_callback(self.goal_response_callback)
        return "ok"

    def publish_speech_once(self, text):
        msg = String()
        msg.data = text
        self.speech_pub.publish(msg)
        self.get_logger().info(f"Published to /communication_hub/robot_speech: {text}")

        # Cancel the timer to prevent repeated publishing
        if hasattr(self, 'timer') and self.timer is not None:
            self.timer.cancel()
            self.timer = None


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected.")
            return
        goal_handle.get_result_async().add_done_callback(self.tts_done_callback)

    def tts_done_callback(self, future):
        result = future.result().result
        if result.error_msg:
            self.get_logger().error(f"TTS Error: {result.error_msg}")
        else:
            self.get_logger().info("TTS completed.")

        client.send("finished")
        client.send("finished")
        client.send("finished")

    def lookup_text(self, msg):
        mapping = {
            "high_explicit_correct_1_1": "Mind assisting during the transfer? If any of it spills, we’ll be under the required dose.",
            "high_explicit_correct_1_2": "Yes.",
            "high_explicit_correct_2_1": "That won’t mix itself.",
            "high_explicit_correct_2_2": "Yes.",
            "high_explicit_correct_3_1": "Another one, please.",
            "high_explicit_correct_3_2": "Yes.",

            "high_implicit_correct_1_1": "Mind assisting during the transfer? If any of it spills, we’ll be under the required dose.",
            "high_implicit_correct_2_1": "That won’t mix itself.",
            "high_implicit_correct_3_1": "Another one, please.",

            "high_explicit_incorrect_1_1": "Mind assisting during the transfer? If any of it spills, we’ll be under the required dose.",
            "high_explicit_incorrect_1_2": "No, please hold the test tube during transfer.",
            "high_explicit_incorrect_2_1": "That won’t mix itself.",
            "high_explicit_incorrect_2_2": "No, please gentlely shake the test tube.",
            "high_explicit_incorrect_3_1": "Another one, please.",
            "high_explicit_incorrect_3_2": "No, please shake the test tube again.",

            "high_implicit_incorrect_1_1": "Mind assisting during the transfer? If any of it spills, we’ll be under the required dose.",
            "high_implicit_incorrect_1_2": "No, please hold the test tube during transfer.",
            "high_implicit_incorrect_2_1": "That won’t mix itself.",
            "high_implicit_incorrect_2_2": "No, please gentlely shake the test tube.",
            "high_implicit_incorrect_3_1": "Another one, please.",
            "high_implicit_incorrect_3_2": "No, please shake the test tube again.",

            "low_explicit_correct_1_1": "Visitors need a strong reference point of what they're about to see. Positioning a classical work at the entrance can provide that foundation.",
            "low_explicit_correct_1_2": "Yes.",
            "low_explicit_correct_2_1": "Having a peaceful work in the middle can balance the sentiment of the tragic one.",
            "low_explicit_correct_2_2": "Yes.",
            "low_explicit_correct_3_1": "And the last one please.",
            "low_explicit_correct_3_2": "Yes.",

            "low_implicit_correct_1_1": "Visitors need a strong reference point of what they're about to see. Positioning a classical work at the entrance can provide that foundation.",
            "low_implicit_correct_2_1": "Having a peaceful work in the middle can balance the sentiment of the tragic one.",
            "low_implicit_correct_3_1": "And the last one please.", 

            "low_explicit_incorrect_1_1": "Visitors need a strong reference point of what they're about to see. Positioning a classical work at the entrance can provide that foundation.",
            "low_explicit_incorrect_1_2": "No, please place the Triumph of Galatea at the top left position.",
            "low_explicit_incorrect_2_1": "Having a peaceful work in the middle can balance the sentiment of the tragic one.",
            "low_explicit_incorrect_2_2": "No, please put the Impression Sunrise at the top middle position.",
            "low_explicit_incorrect_3_1": "And the last one please.",
            "low_explicit_incorrect_3_2": "No, please put the Persistence of Memory at the top right position.",

            "low_implicit_incorrect_1_1": "Visitors need a strong reference point of what they're about to see. Positioning a classical work at the entrance can provide that foundation.",
            "low_implicit_incorrect_1_2": "No, please place the Triumph of Galatea at the top left position.",
            "low_implicit_incorrect_2_1": "Having a peaceful work in the middle can balance the sentiment of the tragic one.",
            "low_implicit_incorrect_2_2": "No, please put the Impression Sunrise at the top middle position.",
            "low_implicit_incorrect_3_1": "And the last one please.",
            "low_implicit_incorrect_3_2": "No, please put the Persistence of Memory at the top right position."
        }
        return mapping.get(msg, None)

def main(args=None):
    rclpy.init(args=args)
    node = TTSBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
