#!/usr/bin/env python3
import rospy
import time
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
        self.last_action_time = time.time()
        self.timeout_duration = 12

        rospy.Timer(rospy.Duration(1.0), self.check_timeout)

        self.script = [
            ("instructor", "high_explicit_correct_1_1", False, None),
            ("tiago", "Want me to hold the test tube for you?", True, None),
            ("instructor", "high_explicit_correct_1_2", False, None),
            ("tiago", "Sure.", False, "hold_test_tube"),
            ("instructor", "high_explicit_correct_2_1", False, None),
            ("tiago", "Want me to shake the test tube?", True, None),
            ("instructor", "high_explicit_correct_2_2", False, None),
            ("tiago", "Got it.", False, "shake_test_tube"),
            ("instructor", "high_explicit_correct_3_1", False, None),
            ("tiago", "Want me to gently shake the test tube again?", True, None),
            ("instructor", "high_explicit_correct_3_2", False, None),
            ("tiago", "(Shake the test tube again)", True, "shake_test_tube")
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
            rospy.sleep(1.0)
            self.listen_pub.publish("start_listen")
            return

        role, text, is_action, action_id = self.script[self.script_index]
        self.status_pub.publish("received")
        self.last_action_time = time.time()

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
                self.action_pub.publish(action_id)
                self.phase = "waiting_tiago_done"
                self.last_action_time = time.time()
            else:
                rospy.sleep(0.5)
                self.script_index += 1
                self.phase = "ready"
                self.advance_script()

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

    def check_timeout(self, event):
        if not self.in_script or self.phase not in ["waiting_tiago_done"]:
            return
        if time.time() - self.last_action_time > self.timeout_duration:
            rospy.logerr(f"[script_mode] Timeout at step {self.script_index}. Advancing.")
            self.phase = "ready"
            self.script_index += 1
            self.advance_script()

if __name__ == "__main__":
    try:
        ScriptedInteraction()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
