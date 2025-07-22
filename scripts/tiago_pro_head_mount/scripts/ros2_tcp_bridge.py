import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from tts_msgs.action import TTS
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
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
        self.head_pub = self.create_publisher(JointTrajectory, '/head_controller/joint_trajectory', 10)

        self.server = TCPServer(port=HEAD_LISTEN_PORT, handler=self.handle_incoming_message)
        self.server.start()

    def handle_incoming_message(self, message, addr):
        text, head_pattern = self.lookup_text(message)
        if not text:
            return "invalid"
        
        if head_pattern:
            self.move_head(head_pattern)

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

    def move_head(self, pattern):
        traj = JointTrajectory()
        traj.joint_names = ['head_1_joint', 'head_2_joint']

        points = []
        if pattern == "right_middle":
            # 先右，再中
            points = [
                ([0.0, 0.0], 0.5),
                ([-0.5, 0.2], 2.0),   # 右
                ([0.0, 0.0], 3.0),   # 中
            ]
        elif pattern == "right_left_middle":
            # 右->左->中
            points = [
                ([0.0, 0.0], 0.5),
                ([-0.5, 0.2], 3.0),   # 右
                ([0.2, 0.0], 5.0),  # 左
                ([0.0, 0.0], 7.0),   # 中
            ]
        elif pattern == "left_right_middle":
            # 左->右->中
            points = [
                ([0.0, 0.0], 0.5),
                ([0.2, 0.0], 1.5),  # 左
                ([-0.5, 0.2], 3.5),   # 右
                ([0.0, 0.0], 5.5),   # 中
            ]
        elif pattern == "left_middle":
            # 先左，再中
            points = [
                ([0.0, 0.0], 0.5),
                ([0.2, 0.0], 3.0),  # 左
                ([0.0, 0.0], 4.0),   # 中
            ]
        else:
            # 默认只回中
            points = [
                ([0.0, 0.0], 0.5)
            ]

        for pos, t in points:
            pt = JointTrajectoryPoint()
            pt.positions = pos
            pt.time_from_start = rclpy.duration.Duration(seconds=t).to_msg()
            traj.points.append(pt)

        self.head_pub.publish(traj)
        self.get_logger().info(f"Published head trajectory: {pattern}")

    def lookup_text(self, msg):
        mapping = {
            "introduction": ("Hi, I'm Diego. It's great to be here with you! Can’t wait to do this together!", "left_middle"),
            "high_explicit_correct_1_1": ("Mind assisting during the transfer? If any of it spills, we’ll be under the required dose.", "right_left_middle"),
            "high_explicit_correct_1_2": ("Yes.", "right_middle"),
            "high_explicit_correct_2_1": ("That won’t mix itself.", "left_right_middle"),
            "high_explicit_correct_2_2": ("Yes.", "right_middle"),
            "high_explicit_correct_3_1": ("Another one, please.", "right_middle"),
            "high_explicit_correct_3_2": ("Yes.", "right_middle"),

            "high_implicit_correct_1_1": ("Mind assisting during the transfer? If any of it spills, we’ll be under the required dose.", "right_left_middle"),
            "high_implicit_correct_2_1": ("That won’t mix itself.", "left_right_middle"),
            "high_implicit_correct_3_1": ("Another one, please.", "right_middle"),

            "high_explicit_incorrect_1_1": ("Mind assisting during the transfer? If any of it spills, we’ll be under the required dose.", "right_left_middle"),
            "high_explicit_incorrect_1_2": ("No, I mean, please hold the test tube during transfer.", "right_middle"),
            "high_explicit_incorrect_2_1": ("That won’t mix itself.", "left_right_middle"),
            "high_explicit_incorrect_2_2": ("No, I mean, please gentlely shake the test tube.", "right_middle"),
            "high_explicit_incorrect_3_1": ("Another one, please.", "right_middle"),
            "high_explicit_incorrect_3_2": ("No, I mean, please shake the test tube again.", "right_middle"),

            "high_implicit_incorrect_1_1": ("Mind assisting during the transfer? If any of it spills, we’ll be under the required dose.", "right_left_middle"),
            "high_implicit_incorrect_1_2": ("No, I mean, please hold the test tube during transfer.", "right_middle"),
            "high_implicit_incorrect_2_1": ("That won’t mix itself.", "left_right_middle"),
            "high_implicit_incorrect_2_2": ("No, I mean, please gentlely shake the test tube.", "right_middle"),
            "high_implicit_incorrect_3_1": ("Another one, please.", "right_middle"),
            "high_implicit_incorrect_3_2": ("No, I mean, please shake the test tube again.", "right_middle"),

            "low_explicit_correct_1_1": ("Probably best to put the strong-smelling one near the exit, less likely to bother customers that way.", "right_middle"),
            "low_explicit_correct_1_2": ("Yes.", "right_middle"),
            "low_explicit_correct_2_1": ("Might make sense to group the veggies together.", "right_middle"),
            "low_explicit_correct_2_2": ("Yes.", "right_middle"),
            "low_explicit_correct_3_1": ("And the last one please.", "right_middle"),
            "low_explicit_correct_3_2": ("Yes.", "right_middle"),

            "low_implicit_correct_1_1": ("Probably best to put the strong-smelling one near the exit, less likely to bother customers that way.", "right_middle"),
            "low_implicit_correct_2_1": ("Might make sense to group the veggies together.", "right_middle"),
            "low_implicit_correct_3_1": ("And the last one please.", "right_middle"), 

            "low_explicit_incorrect_1_1": ("Probably best to put the strong-smelling one near the exit, less likely to bother customers that way.", "right_left_middle"),
            "low_explicit_incorrect_1_2": ("No, I mean, please place durians at the top right position.", "right_middle"),
            "low_explicit_incorrect_2_1": ("Might make sense to group the veggies together.", "right_left_middle"),
            "low_explicit_incorrect_2_2": ("No, I mean, please put potatos at the top middle position.", "right_middle"),
            "low_explicit_incorrect_3_1": ("And the last one please.", "right_middle"),
            "low_explicit_incorrect_3_2": ("No, I mean, please put popcorns at the top left position.", "right_middle"),

            "low_implicit_incorrect_1_1": ("Probably best to put the strong-smelling one near the exit, less likely to bother customers that way.", "right_left_middle"),
            "low_implicit_incorrect_1_2": ("No, I mean, please place durians at the top right position.", "right_middle"),
            "low_implicit_incorrect_2_1": ("Might make sense to group the veggies together.", "right_left_middle"),
            "low_implicit_incorrect_2_2": ("No, I mean, please put potatos at the top middle position.", "right_middle"),
            "low_implicit_incorrect_3_1": ("And the last one please.", "right_middle"),
            "low_implicit_incorrect_3_2": ("No, I mean, please put popcorns at the top left position.", "right_middle")
        }
        return mapping.get(msg, (None, None))

def main(args=None):
    rclpy.init(args=args)
    node = TTSBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
