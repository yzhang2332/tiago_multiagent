#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from queue import Queue, Empty
from threading import Thread, Lock
import time
import yaml
import json
import os
import sys

# Load OpenAI config and client
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from utils.load_config import load_openai_config
import openai

cfg = load_openai_config()
client = openai.OpenAI(api_key=cfg["openai_api_key"])

### ---------- RULE-BASED PART (SignalCoordinator) ---------- ###
class SignalCoordinator:
    def __init__(self):
        self.action_instruction = ""
        self.execution_status = ""
        self.script_agent_status = ""
        self.tts_status = ""
        self.action_agent_status = ""
        self.wizard_intervene = ""

        self.completed_actions = set()
        self.script_triggered = False
        self.script_mode_status = ""
        self.script_trigger_pub = rospy.Publisher("/script_mode_status", String, queue_size=1)


        self.listen_pub = rospy.Publisher("/listen_signal", String, queue_size=10)

        rospy.Subscriber("/script_agent/action_instruction", String, self.handle_action_instruction)
        rospy.Subscriber("/tts_status", String, self.handle_tts_status)
        rospy.Subscriber("/execution_status", String, self.handle_execution_status)
        rospy.Subscriber("/script_agent_status", String, self.handle_script_agent_status)
        rospy.Subscriber("/action_agent_status", String, self.handle_action_agent_status)
        rospy.Subscriber("/wizard_intervene", String, self.handle_wizard_intervene)
        rospy.Subscriber("/script_mode_status", String, self.handle_script_mode)

        rospy.loginfo("SignalCoordinator (rule-based) started.")

    def handle_script_mode(self, msg):
        self.script_mode_status = msg.data.strip().lower()
        # rospy.loginfo(f"[script_mode_status] {self.script_mode_status}")
        if self.script_mode_status == "start_head":
            rospy.loginfo("[Coordinator] Starting scripted Instructor–Tiago flow.")
            rospy.sleep(1)  # Allow time for other statuses to update
            self.completed_actions.clear()
            self.script_triggered = True
        self.evaluate_conditions()

    def handle_wizard_intervene(self, msg):
        self.wizard_intervene = msg.data.strip().lower()
        # rospy.loginfo(f"[wizard_intervene] {self.wizard_intervene}")
        self.evaluate_conditions()

    def handle_action_instruction(self, msg):
        self.action_instruction = msg.data.strip()
        # rospy.loginfo(f"[action_instruction] {self.action_instruction}")

        if "tube" in self.action_instruction and "place" in self.action_instruction:
            self.completed_actions.add("tube")
            rospy.loginfo("[Coordinator] Completed action: tube")
        elif "30" in self.action_instruction and "powder" in self.action_instruction and "place" in self.action_instruction:
            self.completed_actions.add("powder")
            rospy.loginfo("[Coordinator] Completed action: powder")
    
        self.evaluate_conditions()

    def handle_script_agent_status(self, msg):
        self.script_agent_status = msg.data.strip().lower()
        # rospy.loginfo(f"[script_agent_status] {self.script_agent_status}")
        self.evaluate_conditions()

    def handle_action_agent_status(self, msg):
        self.action_agent_status = msg.data.strip().lower()
        # rospy.loginfo(f"[action_agent_status] {self.action_agent_status}")
        self.evaluate_conditions()

    def handle_tts_status(self, msg):
        self.tts_status = msg.data.strip().lower()
        # rospy.loginfo(f"[tts_status] {self.tts_status}")
        self.evaluate_conditions()

    def handle_execution_status(self, msg):
        self.execution_status = msg.data.strip().lower()
        # rospy.loginfo(f"[execution_status] {self.execution_status}")
        self.evaluate_conditions()

    def evaluate_conditions(self):
        rospy.sleep(0.1)  # Allow all statuses to update

        if all(s not in ("received", "failed", "takeover", "need_help", "need_takeover", "start_head", "in_progress", "ready_head") for s in [
            self.script_agent_status,
            self.tts_status,
            self.action_agent_status,
            self.execution_status,
            self.wizard_intervene,
            self.script_mode_status]):
            # rospy.loginfo("SignalCoordinator: No agent is processing. Starting listener.")
            self.send_listen_signal("start_listen")
        else:
            # rospy.loginfo("SignalCoordinator: Agents are busy. Stopping listener.")
            self.send_listen_signal("stop_listen")
        
        # Trigger script if both tube and powder actions completed and execution finished
        if not self.script_triggered and {"tube", "powder"}.issubset(self.completed_actions):
            
            if self.execution_status == "finished":
                rospy.loginfo("[Coordinator] Triggering scripted Instructor–Tiago flow.")
                self.script_trigger_pub.publish("start_head")
                self.completed_actions.clear()  # Optional: reset if this should only run once
                self.script_triggered = True
            else:
                self.script_trigger_pub.publish("ready_head")


    def send_listen_signal(self, signal: str):
        rospy.sleep(0.2)
        self.listen_pub.publish(signal)
        # rospy.loginfo(f"SignalCoordinator published to /listen_signal: {signal}")

### ---------- LLM-BASED PART (WizardAgent) ---------- ###
class WizardAgent:
    def __init__(self):
        # self.listen_pub = rospy.Publisher("/listen_signal", String, queue_size=10)

        self.message_queue = Queue()
        self.buffer_lock = Lock()

        # config = load_openai_config()
        # openai.api_key = config["openai_api_key"]
        self.assistant_id = cfg["wizard_agent_id"]
        if not self.assistant_id:
            rospy.logerr("WizardAgent: assistant_id param missing.")
            rospy.signal_shutdown("Missing assistant ID")
        self.thread = client.beta.threads.create()

        # Load and push static task context
        self.task_data, self.behavior_data, self.aruco_data = self.load_context()
        self.push_initial_context()

        # Subscribe to status/instruction topics
        self.subscribe_topics([
            "/received_utterance",
            "/script_agent/verbal_response",
            "/reference_patch",
            "/script_agent/action_instruction",
            "/action_agent/execute_sequence",
            "wizard_agent/execute_sequence",
            # "/tts_status",
            "/execution_status",
            "/wizard_intervene",
            "/error_log",
            "/script_mode_status"
            # ,
            # "/script_agent_status",
            # "/action_agent_status"
        ])
        
        self.summary_pub = rospy.Publisher("/wizard_agent/summary", String, queue_size=10)

        self.running = True
        self.processor_thread = Thread(target=self.process_loop)
        self.processor_thread.start()

        self.latest_execution_status = "waiting"
        
        rospy.loginfo("WizardAgent (LLM-based) started.")

    def load_context(self):
        with open("../config/high_task.yaml", "r") as f:
            task_data = yaml.safe_load(f)
        with open("../config/behavior_high.yaml", "r") as f:
            behavior_data = yaml.safe_load(f)
        with open("../config/object_aruco_high.yaml", "r") as f:
            aruco_data = yaml.safe_load(f)
        return task_data, behavior_data, aruco_data

    def push_initial_context(self):
        context = f"""
        Task Info:
        {yaml.dump(self.task_data)}

        Robot Behaviour:
        {yaml.dump(self.behavior_data)}

        Object Info:
        {yaml.dump(self.aruco_data)}
        """
        client.beta.threads.messages.create(
            thread_id=self.thread.id,
            role="assistant",
            content=context
        )

    def subscribe_topics(self, topic_list):
        for topic in topic_list:
            rospy.Subscriber(topic, String, self.buffer_message)

    def buffer_message(self, msg):
        # with self.buffer_lock:
        #     self.message_queue.put((time.time(), msg._connection_header['topic'], msg.data.strip()))

        topic = msg._connection_header['topic']
        data = msg.data.strip()

        with self.buffer_lock:
            self.message_queue.put((time.time(), topic, data))

        # Track execution completion
        if topic == "/execution_status":
            self.latest_execution_status = data.lower()


    def process_loop(self):
        rate = rospy.Rate(2)
        while self.running and not rospy.is_shutdown():
            try:
                msgs = self.drain_messages()
                if msgs:
                    decision = self.query_agent(msgs)
                    self.publish_decision(decision)

            except Exception as e:
                rospy.logwarn(f"WizardAgent error: {e}")
            rate.sleep()
    
    def drain_messages(self):
        msgs = []
        while not self.message_queue.empty():
            try:
                msgs.append(self.message_queue.get_nowait())
            except Empty:
                break
        return msgs

    def query_agent(self, msgs):
        prompt = "ROS system messages:\n"
        for _, topic, content in msgs:
            prompt += f"[{topic}] {content}\n"

        client.beta.threads.messages.create(
            thread_id=self.thread.id,
            role="user",
            content=prompt
        )
        run = client.beta.threads.runs.create(
            assistant_id=self.assistant_id,
            thread_id=self.thread.id
        )

        while True:
            run_status = client.beta.threads.runs.retrieve(thread_id=self.thread.id, run_id=run.id)
            if run_status.status == "completed":
                break
            elif run_status.status in ["failed", "cancelled", "expired"]:
                return "stop_listen"
            time.sleep(1)

        messages = client.beta.threads.messages.list(thread_id=self.thread.id)
        return messages.data[0].content[0].text.value.strip().lower()

    def publish_decision(self, signal: str): 
        rospy.loginfo(f"WizardAgent received summary:\n{signal}")
        self.summary_pub.publish(signal)

### ---------- MAIN ---------- ###
if __name__ == "__main__":
    try:
        rospy.init_node("combined_wizard_agent", anonymous=True)
        rule_agent = SignalCoordinator()
        llm_agent = WizardAgent()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
