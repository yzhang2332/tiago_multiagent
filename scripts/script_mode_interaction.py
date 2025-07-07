#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class ScriptedInteraction:
    def __init__(self):
        rospy.init_node("script_mode_interaction", anonymous=True)

        self.to_instructor_pub = rospy.Publisher("/ros1_to_tts", String, queue_size=10)
        self.status_feedback_pub = rospy.Publisher("/script_mode/status", String, queue_size=1)
        self.verbal_pub = rospy.Publisher("/script_agent/verbal_response", String, queue_size=10)
        self.action_pub = rospy.Publisher("/script_agent/action_instruction", String, queue_size=10)
        self.status_pub = rospy.Publisher("/script_agent_status", String, queue_size=10)
        self.listen_pub = rospy.Publisher("/listen_signal", String, queue_size=1)

        rospy.Subscriber("/ros1_feedback", String, self.instructor_feedback_callback)
        rospy.Subscriber("/execution_status", String, self.execution_callback)
        rospy.Subscriber("/start_script_mode", String, self.start_script)
        rospy.Subscriber("/tiago_head_status", String, self.head_status_callback)

        self.latest_head_status = "idle"
        self.script_index = 0
        self.phase = "idle"
        self.in_script = False

        self.script = [
            ("instructor", "high_explicit_correct_1_1", False, None),
            ("tiago", "Want me to hold the test tube for you?", False, None),
            ("instructor", "high_explicit_correct_1_2", False, None),
            ("tiago", "Sure.", True, "hold_test_tube"),
            ("instructor", "high_explicit_correct_2_1", False, None),
            ("tiago", "Want me to shake the test tube?", False, None),
            ("instructor", "high_explicit_correct_2_2", False, None),
            ("tiago", "Got it.", True, "shake_test_tube"),
            ("instructor", "high_explicit_correct_3_1", False, None),
            ("tiago", "Want me to gently shake the test tube again?", False, None),
            ("instructor", "high_explicit_correct_3_2", False, None),
            ("tiago", "(Shake the test tube again)", True, "shake_test_tube_2")
        ]

        rospy.loginfo("[script_mode] ScriptedInteraction node initialised and waiting for start signal.")

    def head_status_callback(self, msg):
        self.latest_head_status = msg.data.strip().lower()

    def start_script(self, msg):
        if self.in_script:
            rospy.logwarn("[script_mode] Script already in progress. Ignoring repeated trigger.")
            return
        if msg.data.strip().lower() == "start":
            rospy.loginfo("[script_mode] Starting scripted flow.")
            self.in_script = True
            self.script_index = 0
            self.phase = "ready"
            self.status_feedback_pub.publish("started")
            self.advance_script()

    def advance_script(self):
        if not self.in_script:
            return

        if self.script_index >= len(self.script):
            rospy.loginfo("[script_mode] Script completed.")
            self.status_pub.publish("finished")
            self.status_feedback_pub.publish("complete")
            self.in_script = False
            self.phase = "idle"
            rospy.sleep(0.5)
            self.listen_pub.publish("start_listen")
            return

        role, text, is_action, action_id = self.script[self.script_index]
        self.status_pub.publish("received")

        rospy.loginfo(f"[script_mode] Step {self.script_index} | Role: {role} | Phase: {self.phase}")

        if role == "instructor":
            rospy.loginfo(f"[script_mode] Publishing instructor TTS: {text}")
            self.to_instructor_pub.publish(text)
            self.phase = "waiting_instructor_done"

        elif role == "tiago":
            rospy.loginfo("[script_mode] Tiago preparing to act...")
            rospy.sleep(12)  # Small pause for realism/synchronisation

            if text:
                rospy.loginfo(f"[script_mode] Tiago verbal: {text}")
                self.verbal_pub.publish(text)

            if is_action and action_id:
                rospy.loginfo(f"[script_mode] Tiago action: {action_id}")
                if action_id == "hold_test_tube":
                    plan_msg = String()
                    plan_msg.data = (
                        '{"plan": ['
                        '{"action": "pickup", "marker_id": 10, "sequence": ['
                        '"search_head", "get_current_arm_position", "open_gripper", "move_to_open", '
                        '"detect_aruco_with_gripper_camera", "move_down", "close_gripper"]}'
                        ']}'
                    )
                    self.action_pub.publish(plan_msg)

                elif action_id == "shake_test_tube":
                    plan_msg = String()
                    plan_msg.data = (
                        '{"plan": ['
                        '{"action": "shake_test_tube", "marker_id":, "sequence": ['
                        '"shake_test_tube"]}'
                        ']}'
                    )
                    self.action_pub.publish(plan_msg)
                
                elif action_id == "shake_test_tube_2":
                    plan_msg = String()
                    plan_msg.data = (
                        '{"plan": ['
                        '{"action": "shake_test_tube", "marker_id":, "sequence": ['
                        '"shake_test_tube"]}, '
                        '{"action": "release", "marker_id":, "sequence": ['
                        '"open_gripper", "move_up", "go_home_position"]}'
                        ']}'
                    )
                    self.action_pub.publish(plan_msg)

                self.phase = "waiting_tiago_done"

    def instructor_feedback_callback(self, msg):
        if self.phase == "waiting_instructor_done" and msg.data.strip().lower() == "finished":
            rospy.loginfo("[script_mode] Instructor finished speaking.")
            self.phase = "ready"
            self.script_index += 1
            rospy.sleep(0.5)
            self.advance_script()

    def execution_callback(self, msg):
        if self.phase == "waiting_tiago_done" and msg.data.strip().lower() == "finished":
            rospy.loginfo("[script_mode] Tiago completed action.")
            self.phase = "ready"
            self.script_index += 1
            rospy.sleep(0.5)
            self.advance_script()

if __name__ == "__main__":
    try:
        ScriptedInteraction()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
