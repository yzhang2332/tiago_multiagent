#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class SignalCoordinator:
    def __init__(self):
        rospy.init_node("signal_coordinator", anonymous=True)

        # Internal state
        self.action_instruction = ""
        self.tts_status = ""
        self.execution_status = ""

        # Publisher
        self.listen_pub = rospy.Publisher("/listen_signal", String, queue_size=10)

        # Subscribers
        rospy.Subscriber("/script_agent/action_instruction", String, self.handle_action_instruction)
        rospy.Subscriber("/tts_status", String, self.handle_tts_status)
        rospy.Subscriber("/execution_status", String, self.handle_execution_status)

        rospy.loginfo("SignalCoordinator is running and listening.")
        rospy.spin()

    def handle_action_instruction(self, msg):
        self.action_instruction = msg.data.strip()
        rospy.loginfo(f"[action_instruction] {self.action_instruction}")
        rospy.sleep(0.2)
        self.evaluate_conditions()

    def handle_tts_status(self, msg):
        self.tts_status = msg.data.strip().lower()
        rospy.loginfo(f"[tts_status] {self.tts_status}")
        self.evaluate_conditions()

    def handle_execution_status(self, msg):
        self.execution_status = msg.data.strip().lower()
        rospy.loginfo(f"[execution_status] {self.execution_status}")
        self.evaluate_conditions()

    def evaluate_conditions(self):
        """Evaluate current state and publish start_listen if conditions are met."""
        if self.action_instruction == "" and self.tts_status == "finished":
            rospy.loginfo("Condition met: no action_instruction and tts_status finished")
            self.send_listen_signal("start_listen")

        elif self.action_instruction != "" and self.execution_status == "finished":
            rospy.loginfo("Condition met: action_instruction present and execution_status finished")
            self.send_listen_signal("start_listen")
        else:
            self.send_listen_signal("stop_listen")

    def send_listen_signal(self, signal: str):
        rospy.sleep(0.2)
        self.listen_pub.publish(signal)
        rospy.loginfo(f"Published to /listen_signal: {signal}")

if __name__ == '__main__':
    try:
        SignalCoordinator()
    except rospy.ROSInterruptException:
        pass
