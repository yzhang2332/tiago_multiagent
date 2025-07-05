#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import tkinter as tk
import threading


class SignalCoordinatorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Signal Coordinator Monitor")
        self.root.geometry("1000x1200")
        self.root.configure(bg="#f0f0f0")

        # Grouped topic sections
        self.sections = {
            "Wizard Agent Summary": ["/wizard_agent/summary"],
            "Speech and User Interaction": [
                "/received_utterance",
                "/script_agent/verbal_response",
                "/script_agent/action_instruction",
                "/tts_status"
            ],
            "Agent Status": [
                "/script_agent_status",
                "/action_agent_status",
                "/execution_status"
            ],
            "Robot Execution / Control": [
                "/action_agent/execute_sequence",
                "/listen_signal"
            ]
        }

        self.latest_msgs = {}
        self.labels = {}

        for section_title, topics in self.sections.items():
            self.add_section(section_title, topics)

        self.root.after(200, self.refresh_gui)

    def add_section(self, title, topics):
        section_frame = tk.LabelFrame(
            self.root, text=title, bg="#f8f9fa",
            font=("Helvetica", 14, "bold"), bd=2,
            relief="groove", padx=6, pady=6
        )
        section_frame.pack(fill="x", padx=14, pady=10)

        for topic in topics:
            self.latest_msgs[topic] = ""

            label_frame = tk.Frame(section_frame, bg="#ffffff")
            label_frame.pack(fill="x", padx=8, pady=5)

            tk.Label(
                label_frame, text=topic, width=30,
                font=("Helvetica", 12, "bold"), anchor="w", bg="#ffffff"
            ).pack(side="left")

            is_long_text = topic in [
                "/wizard_agent/summary",
                "/script_agent/action_instruction",
                "/action_agent/execute_sequence"
            ]
            font = ("Courier", 11) if is_long_text else ("Helvetica", 12)
            wrap = 840

            if is_long_text:
                text_widget = tk.Text(
                    label_frame, height=5, wrap="word", font=font,
                    bg="#fdfdfd", relief="solid", bd=1
                )
                text_widget.pack(fill="x", expand=True, side="left")
                text_widget.insert("1.0", "")
                text_widget.config(state="disabled")
                self.labels[topic] = text_widget
            else:
                label = tk.Label(
                    label_frame, text="", font=font, anchor="w",
                    justify="left", wraplength=wrap, bg="#ffffff"
                )
                label.pack(fill="x", side="left", expand=True)
                self.labels[topic] = label

    def bind_ros(self):
        for section_topics in self.sections.values():
            for topic in section_topics:
                rospy.Subscriber(topic, String, self.update_msg(topic))

    def update_msg(self, topic):
        def callback(msg):
            self.latest_msgs[topic] = msg.data.strip()
        return callback

    def refresh_gui(self):
        for topic, widget in self.labels.items():
            content = self.latest_msgs[topic]
            if isinstance(widget, tk.Text):
                widget.config(state="normal")
                widget.delete("1.0", tk.END)
                widget.insert(tk.END, content)
                widget.config(state="disabled")
            else:
                widget.config(text=content)
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
