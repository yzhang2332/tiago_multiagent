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

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from utils.load_config import load_openai_config

class AgentAction:
    def __init__(self):
        rospy.init_node("agent_action", anonymous=True)

        # Load config
        cfg = load_openai_config()
        openai.api_key = cfg["openai_api_key"]
        self.assistant_id = cfg["action_agent_id"]

        # Load task and style config
        with open("../config/behavior.yaml", "r") as f:
            self.behavior = yaml.safe_load(f)

        with open("../config/object_aruco.yaml", "r") as f:
            self.aruco = yaml.safe_load(f)

        # ROS setup
        rospy.Subscriber("/script_agent/action_instruction", String, self.on_instruction)
        self.execute_pub = rospy.Publisher("/action_agent/execute_sequence", String, queue_size=1)
        self.status_pub  = rospy.Publisher("/action_agent/status", String, queue_size=1)
        self.marker_pub  = rospy.Publisher("/action_agent/target_marker", String, queue_size=1)

        # Create LLM thread
        self.thread = openai.beta.threads.create()
        rospy.loginfo("[agent_action] Ready with thread %s", self.thread.id)

        initial_context = f"""
                            Task Info:
                            {yaml.dump(self.behavior)}

                            Style Info:
                            {yaml.dump(self.aruco)}
                            """
        openai.beta.threads.messages.create(
            thread_id=self.thread.id,
            role="assistant",
            content=initial_context
        )

    def on_instruction(self, msg):
        instr = msg.data.strip()
        if not instr:
            rospy.logwarn("[agent_action] No valid instruction received.")
            self.status_pub.publish("failed:empty_instruction")
            return

        rospy.loginfo(f"[agent_action] Instruction: {instr}")

        # Proceed with thread messaging etc.
        openai.beta.threads.messages.create(
            thread_id=self.thread.id,
            role="user",
            content=instr
        )

        # Run the assistant
        run = openai.beta.threads.runs.create(
            thread_id=self.thread.id,
            assistant_id=self.assistant_id,
            tools=[{"type": "file_search"}]
        )

        while run.status not in ["completed", "failed"]:
            time.sleep(0.1)
            run = openai.beta.threads.runs.retrieve(thread_id=self.thread.id, run_id=run.id)

        if run.status == "failed":
            rospy.logwarn("[agent_action] LLM classification failed.")
            self.status_pub.publish("failed:llm")
            return

        # Get response
        msgs = openai.beta.threads.messages.list(thread_id=self.thread.id)
        response_text = msgs.data[0].content[0].text.value.strip()
        rospy.loginfo(f"[agent_action] LLM raw response: {response_text}")

        # Remove triple backticks if present
        response_text = re.sub(r"^```(?:json)?|```$", "", response_text.strip(), flags=re.MULTILINE).strip()

        try:
            plan = json.loads(response_text)
        except Exception as e:
            rospy.logerr(f"[agent_action] Invalid JSON from LLM: {e}")
            self.status_pub.publish("failed:invalid_json")
            return

        for step in plan:
            self.status_pub.publish(f"decided:{step['action']}")
            if "marker_id" in step:
                self.marker_pub.publish(str(step["marker_id"]))

        self.execute_pub.publish(json.dumps({"plan": plan}))
        rospy.loginfo(f"[agent_action] Published plan: {plan}")

if __name__ == "__main__":
    AgentAction()
    rospy.spin()
