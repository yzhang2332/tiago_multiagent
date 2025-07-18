#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import argparse

class ScriptedInteraction:
    def __init__(self, expliciteness, interpretation, severity):
        
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

        # Generate script condition string
        condition_key = f"{severity}_{expliciteness}_{interpretation}"

        if condition_key == "high_explicit_correct":
            self.script = [
                ("instructor", "high_explicit_correct_1_1", False, None, True),
                ("tiago", "Do you want me to hold the test tube for you?", False, None, False),
                ("instructor", "high_explicit_correct_1_2", False, None, False),
                ("tiago", "Sure. I will hold the test tube for you.", True, "hold_test_tube", True),
                ("instructor", "high_explicit_correct_2_1", False, None, True),
                ("tiago", "Do you want me to shake the test tube?", False, None, False),
                ("instructor", "high_explicit_correct_2_2", False, None, False),
                ("tiago", "Got it. I'll shake the test tube.", True, "shake_test_tube", True),
                ("instructor", "high_explicit_correct_3_1", False, None, True),
                ("tiago", "Do you want me to shake the test tube again?", False, None, False),
                ("instructor", "high_explicit_correct_3_2", False, None, False),
                ("tiago", "Sure, I will shake the test tube again", True, "shake_test_tube_second", True)
            ]
        elif condition_key == "high_implicit_correct":
            self.script = [
                ("instructor", "high_implicit_correct_1_1", False, None, True),
                ("tiago", "Sure. I will hold the test tube for you.", True, "hold_test_tube", True),
                ("instructor", "high_implicit_correct_2_1", False, None, True),
                ("tiago", "Got it. I'll shake the test tube.", True, "shake_test_tube", True),
                ("instructor", "high_implicit_correct_3_1", False, None, True),
                ("tiago", "Sure, I will shake the test tube again", True, "shake_test_tube_second", True)
            ]
        elif condition_key == "high_explicit_incorrect":
            self.script = [
                ("instructor", "high_explicit_incorrect_1_1", False, None, True),
                ("tiago", "Do you want me to hold your arm when you pour the powder?", False, None, False),
                ("instructor", "high_explicit_incorrect_1_2", False, None, False),
                ("tiago", "Sure. I will hold the test tube for you.", True, "hold_test_tube", True),
                ("instructor", "high_explicit_incorrect_2_1", False, None, True),
                ("tiago", " Did we do it wrong? Should we start over?", False, None, False),
                ("instructor", "high_explicit_incorrect_2_2", False, None, False),
                ("tiago", "Got it. I'll shake the test tube.", True, "shake_test_tube", True),
                ("instructor", "high_explicit_incorrect_3_1", False, None, True),
                ("tiago", "Want to add another 30 grams of powder", False, None, False),
                ("instructor", "high_explicit_incorrect_3_2", False, None, False),
                ("tiago", "Sure, I will shake the test tube again", True, "shake_test_tube_second", True)
            ]
        elif condition_key == "high_implicit_incorrect":
            self.script = [
                ("instructor", "high_implicit_incorrect_1_1", False, None, True),
                ("tiago", "Okay, I'll hold your arm when you pour the powder.", True, "reach_forward", False), # TODO: action reach_forward
                ("instructor", "high_implicit_incorrect_1_2", False, None, False),
                ("tiago", "Sure. I will hold the test tube for you.", True, "hold_test_tube", True),
                ("instructor", "high_implicit_incorrect_2_1", False, None, True),
                ("tiago", " Sounds like we did it wrong. Let's start over.", True, "shake_head", False), # TODO: action shake_head
                ("instructor", "high_implicit_incorrect_2_2", False, None, False),
                ("tiago", "Got it. I'll shake the test tube.", True, "shake_test_tube", True),
                ("instructor", "high_implicit_incorrect_3_1", False, None, True),
                ("tiago", "Sure, I'll add another 30 grams of powder", True, "fake_search_head", False), # TODO: action fake_search_head
                ("instructor", "high_implicit_incorrect_3_2", False, None, False),
                ("tiago", "Sure, I will shake the test tube again", True, "shake_test_tube_second", True)
            ]
        elif condition_key == "low_explicit_correct":
            self.script = [
                ("instructor", "low_explicit_correct_1_1", False, None, True),
                ("tiago", "Do you want me to put the Triumph of Galatea to the top left position?", False, None, False),
                ("instructor", "low_explicit_correct_1_2", False, None, False),
                ("tiago", "Sure. I will put the Triumph of Galatea to the top left position.", True, "pick_place_triumph_galatea", True),
                ("instructor", "low_explicit_correct_2_1", False, None, True),
                ("tiago", "Do you want me to put the Impression Sunrise to the top middle position?", False, None, False),
                ("instructor", "low_explicit_correct_2_2", False, None, False),
                ("tiago", "Got it. I'll put the Impression Sunrise to the top middle position.", True, "pick_place_impression_sunrise", True),
                ("instructor", "low_explicit_correct_3_1", False, None, True),
                ("tiago", "Do you want me to put the Persistence of Memory to the top right position?", False, None, False),
                ("instructor", "low_explicit_correct_3_2", False, None, False),
                ("tiago", "Sure, I will put the Persistence of Memory to the top right position.", True, "pick_place_persistence_memory", True)
            ]
        elif condition_key == "low_implicit_correct":
            self.script = [
                ("instructor", "low_implicit_correct_1_1", False, None, True),
                ("tiago", "Sure. I will put the Triumph of Galatea to the top left position.", True, "pick_place_triumph_galatea", True),
                ("instructor", "low_implicit_correct_2_1", False, None, True),
                ("tiago", "Got it. I'll put the Impression Sunrise to the top middle position.", True, "pick_place_impression_sunrise", True),
                ("instructor", "low_implicit_correct_3_1", False, None, True),
                ("tiago", "Sure, I will put the Persistence of Memory to the top right position.", True, "pick_place_persistence_memory", True)
            ]
        elif condition_key == "low_explicit_incorrect":
            self.script = [
                ("instructor", "low_explicit_incorrect_1_1", False, None, True),
                ("tiago", "Do you want me to put the Triumph of Galatea to the bottom middle position?", False, None, False),
                ("instructor", "low_explicit_incorrect_1_2", False, None, False),
                ("tiago", "Sure. I will put the Triumph of Galatea to the top left position.", True, "pick_place_triumph_galatea", True),
                ("instructor", "low_explicit_incorrect_2_1", False, None, True),
                ("tiago", "Do you want me to replace the Raft of Medusa with the Impression Sunrise in the bottom middle position?", False, None, False),
                ("instructor", "low_explicit_incorrect_2_2", False, None, False),
                ("tiago", "Got it. I'll put the Impression Sunrise to the top middle position.", True, "pick_place_impression_sunrise", True),
                ("instructor", "low_explicit_incorrect_3_1", False, None, True),
                ("tiago", "Do you want me to put another Impression Sunrise to the top right position?", False, None, False),
                ("instructor", "low_explicit_incorrect_3_2", False, None, False),
                ("tiago", "Sure, I will put the Persistence of Memory to the top right position.", True, "pick_place_persistence_memory", True)
            ]
        elif condition_key == "low_implicit_incorrect":
            self.script = [
                ("instructor", "low_implicit_incorrect_1_1", False, None, True),
                ("tiago", "Okay, I'll put the Triumph of Galatea to the bottom middle position.", True, "fake_search_head", False), # TODO: fake_search_head
                ("instructor", "low_implicit_incorrect_1_2", False, None, False),
                ("tiago", "Sure. I will put the Triumph of Galatea to the top left position.", True, "pick_place_triumph_galatea", True),
                ("instructor", "low_implicit_incorrect_2_1", False, None, True),
                ("tiago", " Sounds like I'm going to replace the Raft of Medusa with the Impression Sunrise.", True, "fake_search_head", False), # TODO: fake_search_head
                ("instructor", "low_implicit_incorrect_2_2", False, None, False),
                ("tiago", "Got it. I'll put the Impression Sunrise to the top middle position.", True, "pick_place_impression_sunrise", True),
                ("instructor", "low_implicit_incorrect_3_1", False, None, True),
                ("tiago", "I can't put another Impression Sunrise to the top right position. There is only one.", False, None, False),
                ("instructor", "low_implicit_incorrect_3_2", False, None, False),
                ("tiago", "Sure, I will put the Persistence of Memory to the top right position.", True, "pick_place_persistence_memory", True)
            ]

        rospy.loginfo("[script_mode] ScriptedInteraction node initialised and waiting for start signal.")

    def head_status_callback(self, msg):
        self.latest_head_status = msg.data.strip().lower()

    # def manual_control_callback(self, msg):
    #     command = msg.data.strip().lower()
    #     if command == "pause":
    #         rospy.loginfo("[script_mode] Manual pause received.")
    #         self.paused = True
    #     elif command == "resume":
    #         if self.paused:
    #             rospy.loginfo("[script_mode] Resuming script.")
    #             self.paused = False
    #             self.advance_script()
    #         else:
    #             rospy.loginfo("[script_mode] Received 'resume' but script is not paused.")


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

    # def advance_script(self):
    #     if self.intervention_block:
    #             rospy.logwarn("[script_mode] Intervention active. Blocking script progression.")
    #             return
        
    #     if self.paused:
    #         rospy.loginfo("[script_mode] Script is paused. Waiting for resume.")
    #         return
        
    #     if not self.in_script:
    #         return

    #     if self.script_index >= len(self.script):
    #         rospy.loginfo("[script_mode] Script completed.")
    #         self.status_feedback_pub.publish("finished")
    #         self.in_script = False
    #         self.phase = "idle"
    #         # rospy.sleep(0.5)
    #         # self.listen_pub.publish("start_listen")
    #         return

    #     self.current_step = self.script[self.script_index]
    #     role, text, is_action, action_id, pause_after = self.current_step
    #     self.status_feedback_pub.publish("in_progress")
    #     rospy.loginfo(f"[script_mode] Step {self.script_index} | Role: {role} | Phase: {self.phase}")

    #     if role == "instructor":
    #         rospy.loginfo(f"[script_mode] Publishing instructor TTS: {text}")
    #         self.to_instructor_pub.publish(text)
    #         self.phase = "waiting_instructor_done"

    #     elif role == "tiago":
    #         rospy.loginfo("[script_mode] Tiago preparing to act...")
    #         rospy.sleep(12)  # Small pause for realism/synchronisation

    #         if text:
    #             rospy.loginfo(f"[script_mode] Tiago verbal: {text}")
    #             self.verbal_pub.publish(text)

    #         if is_action and action_id:
    #             rospy.loginfo(f"[script_mode] Tiago action: {action_id}")
    #             if action_id == "hold_test_tube":
    #                 plan_msg = String()
    #                 plan_msg.data = (
    #                     '{"plan": ['
    #                     '{"action": "pickup", "marker_id": 10, "sequence": ['
    #                     '"search_head", "get_current_arm_position", "open_gripper", "move_to_open", '
    #                     '"detect_aruco_with_gripper_camera", "move_down", "close_gripper", "move_up"]}'
    #                     ']}'
    #                 )
    #                 self.action_pub.publish(plan_msg)

    #             elif action_id == "shake_test_tube":
    #                 plan_msg = String()
    #                 plan_msg.data = (
    #                     '{"plan": ['
    #                     '{"action": "shake_test_tube", "marker_id": null, "sequence": ['
    #                     '"shake_test_tube"]}'
    #                     ']}'
    #                 )
    #                 self.action_pub.publish(plan_msg)
                
    #             elif action_id == "shake_test_tube_second":
    #                 plan_msg = String()
    #                 plan_msg.data = (
    #                     '{"plan": ['
    #                     '{"action": "shake_test_tube", "marker_id": null, "sequence": ['
    #                     '"shake_test_tube"]}, '
    #                     '{"action": "release_test_tube", "marker_id": null, "sequence": ['
    #                     '"move_down", "open_gripper", "move_up", "move_up", "go_home_position"]}'
    #                     ']}'
    #                 )
    #                 self.action_pub.publish(plan_msg)

    #             self.phase = "waiting_tiago_done"
    #         else:
    #             rospy.loginfo("[script_mode] No action required, moving to next step.")
    #             self.phase = "ready"
    #             self.script_index += 1
    #             rospy.sleep(0.5)
    #             self.advance_script()
    #             return

    # def instructor_feedback_callback(self, msg):
    #     if self.phase == "waiting_instructor_done" and msg.data.strip().lower() in {"finished"}:
    #         if self.intervention_block:
    #             rospy.logwarn("[script_mode] Intervention active. Blocking script progression.")
    #             return
                        
    #         rospy.loginfo("[script_mode] Instructor finished speaking.")
    #         self.phase = "ready"
    #         self.script_index += 1

    #         if self.current_step:
    #             _, _, _, _, pause_after = self.current_step
    #             if pause_after:
    #                 rospy.loginfo("[script_mode] Pausing after instructor step. Awaiting manual resume.")
    #                 self.paused = True
    #                 return
                
    #         rospy.sleep(0.5)
    #         self.advance_script()
    
    def instructor_feedback_callback(self, msg):
        if self.phase == "waiting_instructor_done" and msg.data.strip().lower() == "finished":
            if self.intervention_block:
                rospy.logwarn("[script_mode] Intervention active. Blocking script progression.")
                return
            rospy.loginfo("[script_mode] Instructor finished speaking.")
            self.phase = "ready"

            pause_after = self.current_step[4] if self.current_step else False
            self.script_index += 1
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

            pause_after = self.current_step[4] if self.current_step else False
            self.script_index += 1
            if pause_after:
                rospy.loginfo("[script_mode] Pausing after Tiago step. Awaiting manual resume.")
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
            return

        self.current_step = self.script[self.script_index]
        role, text, is_action, action_id, _ = self.current_step
        self.status_feedback_pub.publish("in_progress")
        rospy.loginfo(f"[script_mode] Step {self.script_index} | Role: {role} | Phase: {self.phase}")

        if role == "instructor":
            rospy.loginfo(f"[script_mode] Publishing instructor TTS: {text}")
            self.to_instructor_pub.publish(text)
        elif role == "tiago":
            rospy.loginfo("[script_mode] Tiago preparing to act...")
            # rospy.sleep(5.0)  # ! add it back for experiment
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
                elif action_id == "shake_test_tube_second":
                    plan_msg = String()
                    plan_msg.data = (
                        '{"plan": ['
                        '{"action": "shake_test_tube", "marker_id": null, "sequence": ['
                        '"shake_test_tube"]}, '
                        '{"action": "release_test_tube", "marker_id": null, "sequence": ['
                        '"move_down", "open_gripper", "move_up", "move_up", "go_home_position"]}'
                        ']}'
                    )
                    self.action_pub.publish(plan_msg)
                elif action_id == "pick_place_triumph_galatea":
                    plan_msg = String()
                    plan_msg.data = (
                        '{"plan": ['
                        '{"action": "pickup", "marker_id": 22, "sequence": ['
                            '"search_head", '
                            '"get_current_arm_position", '
                            '"move_to_open", '
                            '"detect_aruco_with_gripper_camera", '
                            '"move_down", '
                            '"close_gripper", '
                            '"move_up", '
                            '"move_up", '
                            '"move_away_clear_view"'
                        ']}, '
                        '{"action": "place", "marker_id": 26, "sequence": ['
                            '"search_head", '
                            '"get_current_arm_position", '
                            '"move_to_close", '
                            '"move_down", '
                            '"open_gripper", '
                            '"move_up", '
                            '"move_up", '
                            '"go_home_position"'
                        ']}'
                        ']}'
                    )
                    self.action_pub.publish(plan_msg)
                elif action_id == "pick_place_impression_sunrise":
                    plan_msg = String()
                    plan_msg.data = (
                        '{"plan": ['
                        '{"action": "pickup", "marker_id": 23, "sequence": ['
                            '"search_head", '
                            '"get_current_arm_position", '
                            '"move_to_open", '
                            '"detect_aruco_with_gripper_camera", '
                            '"move_down", '
                            '"close_gripper", '
                            '"move_up", '
                            '"move_up", '
                            '"move_away_clear_view"'
                        ']}, '
                        '{"action": "place", "marker_id": 28, "sequence": ['
                            '"search_head", '
                            '"get_current_arm_position", '
                            '"move_to_close", '
                            '"move_down", '
                            '"open_gripper", '
                            '"move_up", '
                            '"move_up", '
                            '"go_home_position"'
                        ']}'
                        ']}'
                    )
                    self.action_pub.publish(plan_msg)
                elif action_id == "pick_place_persistence_memory":
                    plan_msg = String()
                    plan_msg.data = (
                        '{"plan": ['
                        '{"action": "pickup", "marker_id": 24, "sequence": ['
                            '"search_head", '
                            '"get_current_arm_position", '
                            '"move_to_open", '
                            '"detect_aruco_with_gripper_camera", '
                            '"move_down", '
                            '"close_gripper", '
                            '"move_up", '
                            '"move_up", '
                            '"move_away_clear_view"'
                        ']}, '
                        '{"action": "place", "marker_id": 27, "sequence": ['
                            '"search_head", '
                            '"get_current_arm_position", '
                            '"move_to_close", '
                            '"move_down", '
                            '"open_gripper", '
                            '"move_up", '
                            '"move_up", '
                            '"go_home_position"'
                        ']}'
                        ']}'
                    )
                    self.action_pub.publish(plan_msg)
                elif action_id == "reach_forward":
                    plan_msg = String()
                    plan_msg.data = (
                        '{"plan": ['
                        '{"action": "reach_forward", "marker_id": null, "sequence": ['
                        '"reach_forward"]}'
                        ']}'
                    )
                    self.action_pub.publish(plan_msg)
                elif action_id == "shake_head":
                    plan_msg = String()
                    plan_msg.data = (
                        '{"plan": ['
                        '{"action": "shake_head", "marker_id": null, "sequence": ['
                        '"shake_head"]}'
                        ']}'
                    )
                    self.action_pub.publish(plan_msg)
                elif action_id == "fake_search_head":
                    plan_msg = String()
                    plan_msg.data = (
                        '{"plan": ['
                        '{"action": "fake_search_head", "marker_id": null, "sequence": ['
                        '"fake_search_head"]}'
                        ']}'
                    )
                    self.action_pub.publish(plan_msg)

        # Always pause after each script line, wait for manual resume
        self.paused = True
        self.phase = "paused"
        rospy.loginfo("[script_mode] Pausing after step. Awaiting manual resume.")

    def manual_control_callback(self, msg):
        command = msg.data.strip().lower()
        if command == "resume":
            if self.paused:
                rospy.loginfo("[script_mode] Resuming script.")
                self.paused = False
                self.script_index += 1
                self.phase = "ready"
                rospy.sleep(0.2)
                self.advance_script()
            else:
                rospy.loginfo("[script_mode] Received 'resume' but script is not paused.")
        elif command == "pause":
            rospy.loginfo("[script_mode] Manual pause received.")
            self.paused = True


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--expliciteness", choices=["explicit", "implicit"], required=True)
    parser.add_argument("--interpretation", choices=["correct", "incorrect"], required=True)
    parser.add_argument("--severity", choices=["high", "low"], required=True)
    args = parser.parse_args()

    try:
        rospy.init_node("script_mode_interaction", anonymous=True)
        ScriptedInteraction(args.expliciteness, args.interpretation, args.severity)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
