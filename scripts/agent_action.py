#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import openai
import yaml
import time
import sys
import os
import json
import re

# Setup path to utils
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from utils.load_config import load_openai_config

META_KEYWORDS = {
    "pickup": ["pick up", "grab", "bring me", "fetch"],
    "place": ["place", "put", "set down"],
    "hold": ["hold", "stabilise", "stabilize"],
    "rest": ["rest", "home", "reset"]
}

class AgentAction:
    def __init__(self):
        rospy.init_node("agent_action", anonymous=True)

        # Load OpenAI credentials and ActionAgent ID
        config = load_openai_config()
        openai.api_key = config["openai_api_key"]
        self.assistant_id = config["action_agent_id"]

        # Load behavior definitions
        behavior_path = os.path.join(os.path.dirname(__file__), "..", "config", "behavior.yaml")
        with open(behavior_path, "r") as f:
            self.behavior_data = yaml.safe_load(f)

        # Load marker definitions
        aruco_path = os.path.join(os.path.dirname(__file__), "..", "config", "object_aruco.yaml")
        data = yaml.safe_load(open(aruco_path))
        # allow lookup both name→id and id→name
        self.markers = data["markers"]

        # ROS communication
        rospy.Subscriber("/script_agent/action_instruction", String, self.on_instruction)
        self.execute_pub = rospy.Publisher("/action_agent/execute_sequence", String, queue_size=1)
        self.status_pub  = rospy.Publisher("/action_agent/status", String, queue_size=1)
        self.marker_pub  = rospy.Publisher("/action_agent/target_marker", String, queue_size=1)

        # Create persistent LLM thread (for classification fallback if needed)
        self.thread = openai.beta.threads.create()
        rospy.loginfo("[agent_action] Ready with thread %s", self.thread.id)

    def pick_best_marker(self, instr, for_action):
        """
        Choose best marker for 'pickup' vs 'place':
        - 'pickup': exclude any marker whose name contains 'table'
        - 'place':   include *only* markers containing 'table'
        """
        candidates = []
        for m in self.markers:
            name = m["name"].lower()
            is_table = "table" in name

            if for_action == "place" and not is_table:
                # skip non-table markers when placing
                continue
            if for_action == "pickup" and is_table:
                # skip table markers when picking up
                continue
            candidates.append(m)

        # Now score by token overlap as before
        best_id, best_score = None, 0
        tokens_cache = {}
        for m in candidates:
            name = m["name"].lower()
            # cache tokens split once per marker
            tokens = tokens_cache.get(name) or re.split(r"[^\w]+", name)
            tokens = [t for t in tokens if len(t) > 2]
            tokens_cache[name] = tokens

            score = sum(1 for t in tokens if t in instr)
            if score > best_score:
                best_score, best_id = score, m["id"]

        return best_id if best_score > 0 else None

    def detect_meta_actions(self, instr):
        found = []
        for action, kws in META_KEYWORDS.items():
            if any(kw in instr for kw in kws):
                found.append(action)
        return found

    def on_instruction(self, msg):
        instr = msg.data.strip().lower()
        rospy.loginfo(f"[agent_action] Instruction: {instr}")

        # 1) Detect all meta-actions mentioned
        actions = self.detect_meta_actions(instr)
        if not actions:
            rospy.logwarn("[agent_action] No known action keyword found.")
            self.status_pub.publish("failed:no_action")
            return

        plan = []
        # 2) For each action, pick its marker ID and primitive sequence
        for act in actions:
            m_id = self.pick_best_marker(instr, act)
            if m_id:
                rospy.loginfo(f"[agent_action] Matched marker {m_id} for {act}")
                self.marker_pub.publish(str(m_id))
            else:
                rospy.logwarn(f"[agent_action] No marker matched for {act}")
            seq = ( self.behavior_data["meta_actions"].get(act, {}) \
                    .get("sequence") or
                    self.behavior_data["primitives"].get(act, {}) \
                    .get("sequence", []) )
            plan.append({
                "action":     act,
                "marker_id":  m_id,
                "sequence":   seq
            })

        # 3) Publish status and composite plan
        for step in plan:
            self.status_pub.publish(f"decided:{step['action']}")
        self.execute_pub.publish(json.dumps({"plan": plan}))
        rospy.loginfo(f"[agent_action] Published composite plan: {plan}")

if __name__ == "__main__":
    AgentAction()
    rospy.spin()

