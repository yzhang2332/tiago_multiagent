#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import openai
import yaml
import json
import time
import sys
import os
import re
import threading

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from utils.load_config import load_openai_config

class IntegratedAgent:
    def __init__(self):
        rospy.init_node("integrated_agent", anonymous=True)

        cfg = load_openai_config()
        openai.api_key = cfg["openai_api_key"]
        self.assistant_id = cfg["integrated_agent_id"]

        # Load context configs
        with open("../config/prompt_high.yaml", "r") as f:
            self.task_data = yaml.safe_load(f)
        with open("../config/prompt_explicit_correct_high.yaml", "r") as f:
            self.prompt_data = yaml.safe_load(f)
        with open("../config/behavior_high.yaml", "r") as f:
            self.behavior_data = yaml.safe_load(f)
        with open("../config/object_aruco_high.yaml", "r") as f:
            self.aruco_data = yaml.safe_load(f)

        # ROS publishers
        self.status_pub = rospy.Publisher("/script_agent_status", String, queue_size=1)
        self.verbal_pub = rospy.Publisher("/script_agent/verbal_response", String, queue_size=1)
        self.action_pub = rospy.Publisher("/script_agent/action_instruction", String, queue_size=1)
        self.marker_pub = rospy.Publisher("/action_agent/target_marker", String, queue_size=1)
        self.execute_pub = rospy.Publisher("/action_agent/execute_sequence", String, queue_size=1)

        # ROS subscriber
        rospy.Subscriber("/received_utterance", String, self.on_utterance)
        rospy.Subscriber("/reference_patch", String, self.on_reference_patch)


        # Initialise assistant thread
        self.thread = openai.beta.threads.create()
        rospy.loginfo("[integrated_agent] Ready with thread: %s", self.thread.id)

        context = f"""
        Task Info:
        {yaml.dump(self.task_data)}
        {yaml.dump(self.prompt_data)}

        Robot Behaviour:
        {yaml.dump(self.behavior_data)}

        Object Info:
        {yaml.dump(self.aruco_data)}
        """
        openai.beta.threads.messages.create(
            thread_id=self.thread.id,
            role="assistant",
            content=context
        )

        # Reference patch management
        self.reference_patch = None
        self.reference_lock = threading.Lock()

        self.status_pub.publish("waiting")

    def on_reference_patch(self, msg):
        raw = msg.data.strip()
    
        # Ignore known signal words
        if raw in ["finished", "require_patch"]:
            rospy.logdebug(f"[integrated_agent] Ignored control signal on /reference_patch: {raw}")
            return
    
        try:
            data = json.loads(raw)
            references = data.get("reference", [])
            if isinstance(references, list):
                with self.reference_lock:
                    self.reference_patch = references
                    rospy.loginfo(f"[integrated_agent] Stored reference patch: {references}")
        except Exception as e:
            rospy.logwarn(f"[integrated_agent] Invalid reference patch: {e}")

    def on_utterance(self, msg):
        utterance = msg.data.strip()
        if not utterance:
            return

        rospy.loginfo(f"[integrated_agent] Received utterance: {utterance}")
        self.status_pub.publish("received")

        openai.beta.threads.messages.create(
            thread_id=self.thread.id,
            role="user",
            content=utterance
        )

        # Add reference patch to thread if available
        with self.reference_lock:
            ref_patch = self.reference_patch
            self.reference_patch = None  # clear after one use

        if ref_patch:
            ref_text = f"""The participant previously referred to something using words like 'this', 'that', 'here', or 'there'. Their intended meaning has been clarified as:
{json.dumps(ref_patch, indent=2)}
You MUST use these references to resolve any placeholders such as <wizard_input> in the next response only.
"""
            openai.beta.threads.messages.create(
                thread_id=self.thread.id,
                role="user",
                content=ref_text
            )

        try:
            run = openai.beta.threads.runs.create(
                thread_id=self.thread.id,
                assistant_id=self.assistant_id
            )

            while run.status not in ["completed", "failed"]:
                time.sleep(0.1)
                run = openai.beta.threads.runs.retrieve(thread_id=self.thread.id, run_id=run.id)

            if run.status == "failed":
                rospy.logerr("[integrated_agent] Assistant run failed.")
                self.status_pub.publish("failed")
                return

            messages = openai.beta.threads.messages.list(thread_id=self.thread.id)
            reply = messages.data[0].content[0].text.value.strip()

            # Strip JSON fences
            reply = re.sub(r"^```(?:json)?|```$", "", reply.strip(), flags=re.MULTILINE).strip()

            parsed = json.loads(reply)

            verbal = parsed.get("verbal_response", "")
            instruction = parsed.get("action_instruction", "")
            plan = parsed.get("plan", [])

            rospy.loginfo(f"[integrated_agent] Verbal response: {verbal}")
            rospy.loginfo(f"[integrated_agent] Action instruction: {instruction}")
            rospy.loginfo(f"[integrated_agent] Plan: {plan}")

            # Publish outputs
            self.verbal_pub.publish(verbal)
            self.action_pub.publish(instruction)  # ✅ still publishing here
            if isinstance(plan, list):
                marker = next((step["marker_id"] for step in plan if "marker_id" in step), None)
                if marker is not None:
                    self.marker_pub.publish(str(marker))
                self.execute_pub.publish(json.dumps({"plan": plan}))

            self.status_pub.publish("finished")
            rospy.sleep(1.0)
            self.status_pub.publish("waiting")

        except Exception as e:
            rospy.logerr(f"[integrated_agent] Exception: {e}")
            self.status_pub.publish("failed")

if __name__ == "__main__":
    IntegratedAgent()
    rospy.spin()
