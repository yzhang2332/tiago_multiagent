#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import openai
import yaml
import time
import sys
import os
import json

# Setup path to utils
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from utils.load_config import load_openai_config

class AgentScript:
    def __init__(self):
        rospy.init_node("agent_script", anonymous=True)

        # Load OpenAI credentials and ScriptAgent ID
        config = load_openai_config()
        openai.api_key = config["openai_api_key"]
        self.assistant_id = config["script_agent_id"]

        # Load task and style config
        with open("../config/prompt_high.yaml", "r") as f:
            self.task_data = yaml.safe_load(f)

        with open("../config/prompt_explicit_correct_high.yaml", "r") as f:
            self.prompt_data = yaml.safe_load(f)

        # Set up ROS communication
        rospy.Subscriber("/received_utterance", String, self.on_utterance)

        self.pub = rospy.Publisher("/script_agent_status", String, queue_size=1)
        self.pub_verbal = rospy.Publisher("/script_agent/verbal_response", String, queue_size=1)
        self.pub_action = rospy.Publisher("/script_agent/action_instruction", String, queue_size=1)
        # self.pub_nextstep = rospy.Publisher("/script_agent/next_step_prompt", String, queue_size=1)


        # Create memory-preserving thread
        self.thread = openai.beta.threads.create()
        rospy.loginfo("[agent_script] ScriptAgent ready with memory thread: %s", self.thread.id)

        # Inject initial context
        initial_context = f"""
                            Task Info:
                            {yaml.dump(self.task_data)}

                            Style Info:
                            {yaml.dump(self.prompt_data)}
                            """
        openai.beta.threads.messages.create(
            thread_id=self.thread.id,
            role="assistant",
            content=initial_context
        )

        self.pub.publish("waiting")

    def on_utterance(self, msg):
        utterance = msg.data.strip()
        if not utterance:
            return

        rospy.loginfo("[agent_script] Received utterance: %s", utterance)

        self.pub.publish("received")

        # Append message to thread memory
        openai.beta.threads.messages.create(
            thread_id=self.thread.id,
            role="user",
            content=utterance
        )

        # Run ScriptAgent
        try:
            run = openai.beta.threads.runs.create(
                thread_id=self.thread.id,
                assistant_id=self.assistant_id
            )

            # Wait for completion
            while run.status not in ["completed", "failed"]:
                time.sleep(0.1)
                run = openai.beta.threads.runs.retrieve(
                    thread_id=self.thread.id,
                    run_id=run.id
                )

            if run.status == "failed":
                rospy.logerr("[agent_script] ScriptAgent run failed.")
                self.pub.publish("failed")
                return

            # Get latest reply from assistant
            messages = openai.beta.threads.messages.list(thread_id=self.thread.id)
            latest_msg = messages.data[0]

            try:
                # Extract the text content from the assistant's message
                latest = latest_msg.content[0].text.value.strip()
            except Exception as e:
                rospy.logerr(f"[agent_script] Failed to parse assistant response: {e}")
                latest = ""

            rospy.loginfo("[agent_script] ScriptAgent says: %s", latest)
            try:
                if latest.startswith("```json"):
                    latest = latest.replace("```json", "").strip()
                if latest.endswith("```"):
                    latest = latest[:-3].strip()
                    
                parsed = json.loads(latest)

                verbal_response = parsed.get("verbal_response", "")
                action_instruction = parsed.get("action_instruction", "")
                # next_step_prompt = parsed.get("next_step_prompt", "")

                rospy.loginfo("[agent_script] Verbal response: %s", verbal_response)
                rospy.loginfo("[agent_script] Action instruction: %s", action_instruction)
                # rospy.loginfo("[agent_script] Next-step prompt: %s", next_step_prompt)

                # Publish each to a separate topic
                self.pub_verbal.publish(verbal_response)
                self.pub_action.publish(action_instruction)
                
                self.pub.publish("finished")
                rospy.sleep(1.0)
                self.pub.publish("waiting")

            except Exception as e:
                rospy.logerr(f"[agent_script] JSON parsing failed: {e}")
                # self.pub_verbal.publish(latest)  # fallback
                self.pub.publish("failed")

        except Exception as e:
            rospy.logerr("[agent_script] Error during ScriptAgent call: %s", str(e))
            self.pub.publish("failed")

if __name__ == "__main__":
    AgentScript()
    rospy.spin()
