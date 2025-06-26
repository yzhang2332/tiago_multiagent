#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import tkinter as tk
import threading

class SignalCoordinatorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Signal Coordinator Monitor")
        self.root.geometry("600x450")

        # Shared state
        self.latest_msgs = {
            "/listen_signal": "",
            "/received_utterance": "",
            "/script_agent/verbal_response": "",
            "/tts_status": "",
            "/script_agent/action_instruction": "",
            "/action_agent/execute_sequence": "",
            "/execution_status": ""
        }

        # GUI labels for each topic
        self.labels = {}
        for topic in self.latest_msgs:
            label = tk.Label(root, text=f"{topic}:\n", font=("Helvetica", 11), anchor="w", justify="left")
            label.pack(fill="x", padx=10, pady=4)
            self.labels[topic] = label

        # Start periodic GUI updates
        self.root.after(200, self.refresh_gui)

    def bind_ros(self):
        rospy.Subscriber("/script_agent/action_instruction", String, self.update_msg("/script_agent/action_instruction"))
        rospy.Subscriber("/tts_status", String, self.update_msg("/tts_status"))
        rospy.Subscriber("/execution_status", String, self.update_msg("/execution_status"))
        rospy.Subscriber("/received_utterance", String, self.update_msg("/received_utterance"))
        rospy.Subscriber("/script_agent/verbal_response", String, self.update_msg("/script_agent/verbal_response"))
        rospy.Subscriber("/action_agent/execute_sequence", String, self.update_msg("/action_agent/execute_sequence"))
        rospy.Subscriber("/listen_signal", String, self.update_msg("/listen_signal"))

    def update_msg(self, topic):
        def callback(msg):
            self.latest_msgs[topic] = msg.data.strip()
        return callback

    def refresh_gui(self):
        for topic, label in self.labels.items():
            label.config(text=f"{topic}: {self.latest_msgs[topic]}")
        self.root.after(200, self.refresh_gui)

def ros_thread():
    rospy.init_node("signal_coordinator_gui", anonymous=True, disable_signals=True)
    gui.bind_ros()
    rospy.spin()

if __name__ == '__main__':
    root = tk.Tk()
    gui = SignalCoordinatorGUI(root)
    threading.Thread(target=ros_thread, daemon=True).start()
    root.mainloop()
