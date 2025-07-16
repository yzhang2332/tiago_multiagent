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
import argparse

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from utils.load_config import load_openai_config

class IntegratedAgent:
    def __init__(self, expliciteness, interpretation, severity):
        rospy.init_node("integrated_agent", anonymous=True)

        self.expliciteness = expliciteness
        self.interpretation = interpretation
        self.severity = severity

        cfg = load_openai_config()
        openai.api_key = cfg["openai_api_key"]
        # self.assistant_id = cfg["integrated_agent_id"]

        agent_key = f"integrated_agent_{expliciteness}_{interpretation}_{severity}_id" # e.g. integrated_agent_explicit_correct_high_id
        self.assistant_id = cfg.get(agent_key)
        if not self.assistant_id:
            rospy.logerr(f"[integrated_agent] Missing assistant ID for condition: {agent_key}")
            sys.exit(1)

        # Load context configs
        task_file = f"../config/task_{severity}.yaml"
        prompt_file = f"../config/prompt_integrated_{expliciteness}_{interpretation}_{severity}.yaml"
        behavior_file = f"../config/behavior_{severity}.yaml"
        aruco_file = f"../config/object_aruco_{severity}.yaml"
        
        try:
            with open(task_file, "r") as f:
                self.task_data = yaml.safe_load(f)
            with open(prompt_file, "r") as f:
                self.prompt_data = yaml.safe_load(f)
            with open(behavior_file, "r") as f:
                self.behavior_data = yaml.safe_load(f)
            with open(aruco_file, "r") as f:
                self.aruco_data = yaml.safe_load(f)
        except FileNotFoundError as e:
            rospy.logerr(f"[integrated_agent] YAML file not found: {e}")
            sys.exit(1)

        # ROS publishers
        self.status_pub = rospy.Publisher("/script_agent_status", String, queue_size=1)
        self.verbal_pub = rospy.Publisher("/script_agent/verbal_response", String, queue_size=1)
        self.action_pub = rospy.Publisher("/script_agent/action_instruction", String, queue_size=1)
        self.marker_pub = rospy.Publisher("/action_agent/target_marker", String, queue_size=1)
        self.execute_pub = rospy.Publisher("/action_agent/execute_sequence", String, queue_size=1)
        self.reference_patch_pub = rospy.Publisher("/reference_patch", String, queue_size=1)

        self.status_pub.publish("waiting")
        
        # Reference patch management
        self.reference_patch = None
        self.reference_lock = threading.Lock()
        self.patch_event = threading.Event()

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

        # ROS subscriber
        rospy.Subscriber("/received_utterance", String, self.on_utterance)
        rospy.Subscriber("/reference_patch", String, self.on_reference_patch)


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
                    self.patch_event.set()
                    rospy.loginfo(f"[integrated_agent] Stored reference patch: {references}")
        except Exception as e:
            rospy.logwarn(f"[integrated_agent] Invalid reference patch: {e}")

    def on_utterance(self, msg):
        utterance = msg.data.strip()
        if not utterance:
            return

        rospy.loginfo(f"[integrated_agent] Received utterance: {utterance}")
        self.status_pub.publish("received")

        # Clear previous patch
        with self.reference_lock:
            self.patch_event.clear()
            ref_patch = None
            self.reference_patch = None

        DEICTIC_TERMS = ["this", "that", "here", "there"]

        def contains_deictic(utterance):
            return any(term in utterance.lower() for term in DEICTIC_TERMS)

        # If implicit + placeholder → wait for patch BEFORE sending to LLM
        if self.expliciteness == "implicit" and contains_deictic(utterance):
            rospy.loginfo("[integrated_agent] Implicit + <wizard_input> → waiting for patch before sending to LLM.")
            self.reference_patch_pub.publish("require_patch")

            timeout = 100
            if self.patch_event.wait(timeout):
                with self.reference_lock:
                    ref_patch = self.reference_patch
                    self.reference_patch = None
            else:
                rospy.logwarn("[integrated_agent] Timed out waiting for reference patch.")
                self.status_pub.publish("failed")
                return

            rospy.loginfo(f"[integrated_agent] Using patch: {ref_patch}")

            # Construct combined content
            ref_text = f"""The participant referred to something using words like 'this', 'that', 'here', or 'there'. Their intended meaning is:
    {json.dumps(ref_patch, indent=2)}
    Use these references to resolve any <wizard_input> placeholders in the next response only."""
            
            # Add patch as first message, then the utterance
            openai.beta.threads.messages.create(thread_id=self.thread.id, role="user", content=ref_text)
            openai.beta.threads.messages.create(thread_id=self.thread.id, role="user", content=utterance)

        else:
            # Explicit: send utterance first
            openai.beta.threads.messages.create(thread_id=self.thread.id, role="user", content=utterance)

            # If patch available, attach it after
            with self.reference_lock:
                ref_patch = self.reference_patch
                self.reference_patch = None

            if self.expliciteness == "explicit" and ref_patch:
                rospy.loginfo(f"[integrated_agent] Explicit + patch available → sending it to LLM: {ref_patch}")
                ref_text = f"""The participant previously referred to something using words like 'this', 'that', 'here', or 'there'. Their intended meaning has been clarified as:
    {json.dumps(ref_patch, indent=2)}
    Use these references to resolve any <wizard_input> placeholders in the next response only."""
                openai.beta.threads.messages.create(thread_id=self.thread.id, role="user", content=ref_text)

        # Proceed with OpenAI call as before...
        try:
            run = openai.beta.threads.runs.create(thread_id=self.thread.id, assistant_id=self.assistant_id)
            while run.status not in ["completed", "failed"]:
                time.sleep(0.1)
                run = openai.beta.threads.runs.retrieve(thread_id=self.thread.id, run_id=run.id)

            if run.status == "failed":
                rospy.logerr("[integrated_agent] Assistant run failed.")
                self.status_pub.publish("failed")
                return

            messages = openai.beta.threads.messages.list(thread_id=self.thread.id)
            reply = messages.data[0].content[0].text.value.strip()
            reply = re.sub(r"^```(?:json)?|```$", "", reply.strip(), flags=re.MULTILINE).strip()
            parsed = json.loads(reply)

            verbal = parsed.get("verbal_response", "")
            instruction = parsed.get("action_instruction", "")
            plan = parsed.get("plan", [])

            rospy.loginfo(f"[integrated_agent] Verbal response: {verbal}")
            rospy.loginfo(f"[integrated_agent] Action instruction: {instruction}")
            rospy.loginfo(f"[integrated_agent] Plan: {plan}")

            self.verbal_pub.publish(verbal)
            self.action_pub.publish(instruction)
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
    parser = argparse.ArgumentParser()
    parser.add_argument("--expliciteness", choices=["explicit", "implicit"], required=True)
    parser.add_argument("--interpretation", choices=["correct", "incorrect"], required=True)
    parser.add_argument("--severity", choices=["high", "low"], required=True)

    args = parser.parse_args()

    IntegratedAgent(args.expliciteness, args.interpretation, args.severity)
    rospy.spin()
