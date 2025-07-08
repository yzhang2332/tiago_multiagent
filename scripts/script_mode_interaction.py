#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class ScriptedInteraction:
    def __init__(self):
        
        self.to_instructor_pub = rospy.Publisher("/ros1_to_tts", String, queue_size=10)
        self.status_feedback_pub = rospy.Publisher("/script_mode_status", String, queue_size=1)
        self.verbal_pub = rospy.Publisher("/script_agent/verbal_response", String, queue_size=10)
        self.action_pub = rospy.Publisher("/action_agent/execute_sequence", String, queue_size=10)
        self.listen_pub = rospy.Publisher("/listen_signal", String, queue_size=1)

        rospy.Subscriber("/ros1_feedback", String, self.instructor_feedback_callback)
        rospy.Subscriber("/execution_status", String, self.execution_callback)
        rospy.Subscriber("/script_mode_status", String, self.start_script)
        rospy.Subscriber("/tiago_head_status", String, self.head_status_callback)
        rospy.Subscriber("/wizard_intervene", String, self.wizard_intervene_callback)
        rospy.Subscriber("/script_manual_control", String, self.manual_control_callback)


        self.latest_head_status = "idle"
        self.script_index = 0
        self.phase = "idle"
        self.in_script = False
        self.intervention_block = False
        self.latest_execution_status = None
        self.paused = False
        self.current_step = None

        rospy.sleep(0.5)
        self.status_feedback_pub.publish("waiting")

        self.script = [
            ("instructor", "high_explicit_correct_1_1", False, None, True),
            ("tiago", "Want me to hold the test tube for you?", False, None, False),
            ("instructor", "high_explicit_correct_1_2", False, None, False),
            ("tiago", "Sure. I will hold the test tube for you.", True, "hold_test_tube", True),
            ("instructor", "high_explicit_correct_2_1", False, None, True),
            ("tiago", "Want me to shake the test tube?", False, None, False),
            ("instructor", "high_explicit_correct_2_2", False, None, False),
            ("tiago", "Got it. I'll shake the test tube.", True, "shake_test_tube", True),
            ("instructor", "high_explicit_correct_3_1", False, None, True),
            ("tiago", "Want me to shake the test tube again?", False, None, False),
            ("instructor", "high_explicit_correct_3_2", False, None, False),
            ("tiago", "Sure, I will shake the test tube again", True, "shake_test_tube_2", True)
        ]

        rospy.loginfo("[script_mode] ScriptedInteraction node initialised and waiting for start signal.")

    def head_status_callback(self, msg):
        self.latest_head_status = msg.data.strip().lower()

    def manual_control_callback(self, msg):
        command = msg.data.strip().lower()
        if command == "pause":
            rospy.loginfo("[script_mode] Manual pause received.")
            self.paused = True
        elif command == "resume":
            if self.paused:
                rospy.loginfo("[script_mode] Resuming script.")
                self.paused = False
                self.advance_script()

    def start_script(self, msg):
        if self.in_script:
            rospy.logwarn("[script_mode] Script already in progress. Ignoring repeated trigger.")
            return
        if msg.data.strip().lower() == "start_head":
            rospy.loginfo("[script_mode] Starting scripted flow.")
            self.in_script = True
            self.script_index = 0
            self.phase = "ready"
            self.status_feedback_pub.publish("in_progress")
            self.advance_script()

    def advance_script(self):
        if self.intervention_block:
                rospy.logwarn("[script_mode] Intervention active. Blocking script progression.")
                return
        
        if self.paused:
            rospy.loginfo("[script_mode] Script is paused. Waiting for resume.")
            return
        
        if not self.in_script:
            return

        if self.script_index >= len(self.script):
            rospy.loginfo("[script_mode] Script completed.")
            self.status_feedback_pub.publish("finished")
            self.in_script = False
            self.phase = "idle"
            # rospy.sleep(0.5)
            # self.listen_pub.publish("start_listen")
            return

        self.current_step = self.script[self.script_index]
        role, text, is_action, action_id, pause_after = self.current_step
        self.status_feedback_pub.publish("in_progress")
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
                        '"detect_aruco_with_gripper_camera", "move_down", "close_gripper", "move_up"]}'
                        ']}'
                    )
                    self.action_pub.publish(plan_msg)

                elif action_id == "shake_test_tube":
                    plan_msg = String()
                    plan_msg.data = (
                        '{"plan": ['
                        '{"action": "shake_test_tube", "marker_id": null, "sequence": ['
                        '"shake_test_tube"]}'
                        ']}'
                    )
                    self.action_pub.publish(plan_msg)
                
                elif action_id == "shake_test_tube_2":
                    plan_msg = String()
                    plan_msg.data = (
                        '{"plan": ['
                        '{"action": "shake_test_tube", "marker_id": null, "sequence": ['
                        '"shake_test_tube"]}, '
                        '{"action": "release", "marker_id": null, "sequence": ['
                        '"move_down", "open_gripper", "move_up", "move_up", "go_home_position"]}'
                        ']}'
                    )
                    self.action_pub.publish(plan_msg)

                self.phase = "waiting_tiago_done"
            else:
                rospy.loginfo("[script_mode] No action required, moving to next step.")
                self.phase = "ready"
                self.script_index += 1
                rospy.sleep(0.5)
                self.advance_script()
                return

    def instructor_feedback_callback(self, msg):
        if self.phase == "waiting_instructor_done" and msg.data.strip().lower() in {"finished"}:
            if self.intervention_block:
                rospy.logwarn("[script_mode] Intervention active. Blocking script progression.")
                return
                        
            rospy.loginfo("[script_mode] Instructor finished speaking.")
            self.phase = "ready"
            self.script_index += 1

            if self.current_step:
                _, _, _, _, pause_after = self.current_step
                if pause_after:
                    rospy.loginfo("[script_mode] Pausing after instructor step. Awaiting manual resume.")
                    self.paused = True
                    return
                
            rospy.sleep(0.5)
            self.advance_script()

    def execution_callback(self, msg):
        status = msg.data.strip().lower()
        self.latest_execution_status = status
        if self.phase == "waiting_tiago_done" and status in {"finished", "takeover_finished", "takeover_waiting"}:
            if self.intervention_block:
                rospy.logwarn("[script_mode] Intervention active. Blocking script progression.")
                return
            
            rospy.loginfo("[script_mode] Tiago completed action.")
            self.phase = "ready"
            self.script_index += 1

            if self.current_step:
                _, _, _, _, pause_after = self.current_step
                if pause_after:
                    rospy.loginfo("[script_mode] Pausing after instructor step. Awaiting manual resume.")
                    self.paused = True
                    return
                
            rospy.sleep(0.5)
            self.advance_script()
    
    def wizard_intervene_callback(self, msg):
        data = msg.data.strip().lower()
        if data in {"takeover", "need_takeover", "need_help"}:
            rospy.logwarn(f"[script_mode] Intervention triggered: {data}")
            self.intervention_block = True
        else:
            self.intervention_block = False


if __name__ == "__main__":
    try:
        rospy.init_node("script_mode_interaction", anonymous=True)
        ScriptedInteraction()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
