#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import tkinter as tk
import threading
import json
import os
import subprocess
from collections import defaultdict

# === Load configuration ===
config_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../config"))
with open(os.path.join(config_dir, "object_aruco_high.json"), "r") as f:
    aruco_data = json.load(f)
with open(os.path.join(config_dir, "behavior_high.json"), "r") as f:
    behavior_data = json.load(f)

PRIMITIVES = list(behavior_data["primitives"].keys())
META_ACTIONS = list(behavior_data["meta_actions"].keys())

# === Experiment Config Options and Commands ===
EXPERIMENT_CONFIG = {
    "Condition": ["explicit", "implicit", "correct", "incorrect"],
    "Task": ["high", "low"]
}
COMMANDS = {
    "Mount pro head": ["bash", "mount_pro_head.sh"],
    "Wizard agent": ["python", "agent_wizard.py"],
    "Script and action agent": ["python", "agent_integrate.py"],
    "TTS service": ["python", "tts_service.py"],
    "Start execution": ["python", "execute_control.py"],
    "Run pro head": ["bash", "run_pro_head.sh"],
    "Start script mode": ["python", "script_mode_interaction.py"],
    "Start phone display (app)": ["python", "phone_display/app.py"],
    "Start phone display (ros_client)": ["python", "phone_display/ros_client.py"],
    "Record rosbag": ["python", "record_rosbag.py"],
    "Speech interface": ["python", "speech_to_text.py"]
}
thread_status = defaultdict(lambda: "inactive")
thread_refs = {}
process_refs = {}
status_labels = {}
selection = {"Condition": [], "Task": None}

def run_command(label, cmd):
    def target():
        thread_status[label] = "active"
        update_status(label)
        try:
            proc = subprocess.Popen(cmd)
            process_refs[label] = proc
            proc.wait()
        except Exception as e:
            print(f"Error running {label}: {e}")
        thread_status[label] = "inactive"
        update_status(label)
    t = threading.Thread(target=target)
    thread_refs[label] = t
    t.start()

def update_status(label):
    if label in status_labels:
        status_labels[label].config(text=thread_status[label], fg="green" if thread_status[label] == "active" else "grey")

def toggle_condition(val, config_display):
    if val in selection["Condition"]:
        selection["Condition"].remove(val)
    elif len(selection["Condition"]) < 2:
        selection["Condition"].append(val)
    update_selected_config(config_display)

def select_task(val, config_display):
    selection["Task"] = val
    update_selected_config(config_display)

def update_selected_config(config_display):
    config_display.config(text=f"Condition: {selection['Condition']} | Task: {selection['Task']}")

def build_experiment_column(root):
    leftmost = tk.LabelFrame(root, text="Experiment Config", padx=8, pady=8, width=350)
    leftmost.pack(side="left", fill="y", padx=5, pady=5)
    leftmost.pack_propagate(False)

    config_display = tk.Label(leftmost, text="Condition: [] | Task: None", wraplength=240, justify="left")
    config_display.pack(pady=10)

    tk.Label(leftmost, text="Select Conditions", font=("Helvetica", 11, "bold")).pack()
    for cond in EXPERIMENT_CONFIG["Condition"]:
        b = tk.Button(leftmost, text=cond, width=20, command=lambda v=cond: toggle_condition(v, config_display))
        b.pack()

    tk.Label(leftmost, text="---").pack(pady=4)

    tk.Label(leftmost, text="Select Task", font=("Helvetica", 11, "bold")).pack()
    for task in EXPERIMENT_CONFIG["Task"]:
        b = tk.Button(leftmost, text=task, width=20, command=lambda v=task: select_task(v, config_display))
        b.pack()

    tk.Label(leftmost, text="---").pack(pady=6)

    tk.Label(leftmost, text="Run Processes", font=("Helvetica", 11, "bold")).pack()
    for label, cmd in COMMANDS.items():
        row = tk.Frame(leftmost)
        row.pack(fill="x", pady=2)

        actual_cmd = cmd.copy()
        if label == "Start script and action agent":
            def run_script_action_agent(l=label, c=cmd):
                args = selection["Condition"] + ([selection["Task"]] if selection["Task"] else [])
                run_command(l, c + args)
            btn = tk.Button(row, text=label, width=25, anchor="w", command=run_script_action_agent)
        else:
            btn = tk.Button(row, text=label, width=25, anchor="w", command=lambda l=label, c=cmd: run_command(l, c))
        btn.pack(side="left")

        status = tk.Label(row, text=thread_status[label], fg="grey")
        status.pack(side="right")
        status_labels[label] = status

    return leftmost

class SignalCoordinatorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Signal Coordinator Monitor")
        self.root.geometry("1800x1400")
        self.root.configure(bg="#f0f0f0")

        # ROS publishers
        self.pub_intervene = rospy.Publisher("/wizard_intervene", String, queue_size=1)
        self.pub_execute = rospy.Publisher("/wizard_agent/execute_sequence", String, queue_size=1)
        self.pub_patch = rospy.Publisher("/aruco_patch", String, queue_size=1)
        self.pub_reference_patch = rospy.Publisher("/reference_patch", String, queue_size=1)

        self.latest_msgs = {}
        self.labels = {}
        self.selected_primitives = []
        self.selected_marker_names = []
        self.selected_marker_ids = []



        self.sections = {
            "Wizard Agent Summary": ["/wizard_agent/summary"],
            "Speech and User Interaction": [
                "/listen_signal",  # moved up
                "/received_utterance",
                "/script_agent/verbal_response",
                "/script_agent/action_instruction",
                "/tts_status"
            ],
            "Agent Status": [
                "/script_agent_status",
                "/action_agent_status"
            ],
            "Robot Execution / Control": [
                "/action_agent/execute_sequence",
                "/wizard_agent/execute_sequence",  # new
                "/execution_status"  # moved below
            ],
            "System Interventions": [
                "/reference_patch",
                "/wizard_intervene",
                "/error_log"
            ]
        }

        self.message_times = {}

        # === Layout: Left | Center | Right ===
        main_frame = tk.Frame(self.root)
        main_frame.pack(fill="both", expand=True)

        self.exp_column = build_experiment_column(main_frame)

        self.left_frame = tk.Frame(main_frame, width=800)
        self.left_frame.pack(side="left", fill="both", expand=True)
        self.left_frame.pack_propagate(False)

        self.right_frame = tk.LabelFrame(main_frame, text="Wizard Controls", font=("Helvetica", 14, "bold"), padx=8, pady=8, width=700)
        self.right_frame.pack(side="right", fill="y", padx=10, pady=10)
        self.right_frame.pack_propagate(False)

        for section_title, topics in self.sections.items():
            self.add_section(section_title, topics)

        self.add_wizard_controls()
        self.root.after(200, self.refresh_gui)

    def add_section(self, title, topics):
        section_frame = tk.LabelFrame(
            self.left_frame, text=title, bg="#f8f9fa",
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
                "/action_agent/execute_sequence",
                "/wizard_agent/execute_sequence"
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

    def add_wizard_controls(self):
        # --- Takeover ---
        tk.Button(
            self.right_frame, text="Takeover", bg="#d9534f", fg="white",
            font=("Helvetica", 12, "bold"), command=self.send_takeover
        ).pack(pady=4, fill="x")

        # === Grouped Action + Marker Selection Side by Side ===
        action_marker_frame = tk.Frame(self.right_frame)
        action_marker_frame.pack(pady=10, fill="x")

        # --- Actions ---
        action_frame = tk.LabelFrame(action_marker_frame, text="Select Action", font=("Helvetica", 12, "bold"), padx=4, pady=4)
        action_frame.pack(side="left", fill="both", expand=True, padx=5)

        self.primitive_buttons = []
        for name in META_ACTIONS + PRIMITIVES:
            btn = tk.Button(action_frame, text=name,
                            command=lambda n=name: self.toggle_primitive(n),
                            font=("Helvetica", 11), bg="#ddffee", relief="raised")
            btn.pack(fill="x", pady=1)
            self.primitive_buttons.append((btn, name))

        # --- Markers ---
        marker_frame = tk.LabelFrame(action_marker_frame, text="Select Marker", font=("Helvetica", 12, "bold"), padx=4, pady=4)
        marker_frame.pack(side="right", fill="both", expand=True, padx=5)

        self.marker_buttons = []
        for marker in aruco_data["markers"]:
            mid = marker["id"]
            name = marker["name"]
            btn = tk.Button(marker_frame, text=f"{name} (ID: {mid})",
                            command=lambda m=mid: self.select_marker(m),
                            wraplength=240, font=("Helvetica", 11),
                            bg="#e0e0ff", relief="raised")
            btn.pack(fill="x", pady=1)
            self.marker_buttons.append((btn, mid))

        # --- Update & Verbal Patch Buttons Side by Side ---
        button_row = tk.Frame(self.right_frame)
        button_row.pack(pady=(12, 8), fill="x")

        tk.Button(
            button_row, text="Update & Send", bg="#5cb85c", fg="white",
            font=("Helvetica", 12, "bold"), command=self.update_and_takeover,
            width=18
        ).pack(side="left", padx=5, fill="x", expand=True)

        tk.Button(
            button_row, text="Verbal Patch", bg="#428bca", fg="white",
            font=("Helvetica", 12, "bold"), command=self.send_verbal_patch,
            width=18
        ).pack(side="left", padx=5, fill="x", expand=True)

        # --- Stop Takeover Button ---
        tk.Button(
            self.right_frame, text="Stop Takeover", bg="#f0ad4e", fg="white",
            font=("Helvetica", 12, "bold"), command=self.send_stop_takeover
        ).pack(pady=(0, 12), fill="x")


    def toggle_primitive(self, name):
        if name in self.selected_primitives:
            self.selected_primitives.remove(name)
        else:
            self.selected_primitives.append(name)

        for btn, pname in self.primitive_buttons:
            # btn.config(relief="sunken" if pname in self.selected_primitives else "raised")
            if pname in self.selected_primitives:
                btn.config(relief="sunken", font=("Helvetica", 11, "bold"), bg="#aaffaa")
            else:
                btn.config(relief="raised", font=("Helvetica", 11), bg="#ddffee")

    def select_marker(self, marker_id):
        if marker_id not in self.selected_marker_ids:
            self.selected_marker_ids.append(marker_id)

            name = next((marker["name"] for marker in aruco_data["markers"] if marker["id"] == marker_id), str(marker_id))
            self.selected_marker_names.append(name)

        # Highlight all selected, emphasize the last one
        for btn, mid in self.marker_buttons:
            if mid in self.selected_marker_ids:
                if mid == self.selected_marker_ids[-1]:  # last clicked
                    btn.config(relief="sunken", font=("Helvetica", 11, "bold"), bg="#3366cc", fg="white")
                else:
                    btn.config(relief="sunken", font=("Helvetica", 11), bg="#aaccff", fg="black")
            else:
                btn.config(relief="raised", font=("Helvetica", 11), bg="#e0e0ff", fg="black")


    def send_takeover(self):
        self.pub_intervene.publish("takeover")
    
    def send_stop_takeover(self):
        pub = rospy.Publisher("/execution_status", String, queue_size=1)
        rospy.sleep(0.1)
        pub.publish("finished")
        self.pub_intervene.publish("stop_takeover")

    def update_and_takeover(self):
        self.send_takeover()
        if self.selected_primitives:
            plan = []
            last_marker_id = self.selected_marker_ids[-1] if self.selected_marker_ids else None
            for p in self.selected_primitives:
                if p in META_ACTIONS:
                    sequence = behavior_data["meta_actions"][p]["sequence"]
                else:
                    sequence = [p]
                plan.append({
                    "action": p,
                    "marker_id": last_marker_id,
                    "sequence": sequence
                })
            msg = String(data=json.dumps({"plan": plan}))
            self.pub_execute.publish(msg)
        elif self.selected_marker_ids:
            last_marker_id = self.selected_marker_ids[-1]
            msg = String(data=json.dumps({"marker_id": last_marker_id}))
            self.pub_patch.publish(msg)

        # Clear selection
        self.selected_primitives.clear()
        self.selected_marker_ids.clear()
        self.selected_marker_names.clear()
        for btn, _ in self.marker_buttons:
            btn.config(relief="raised", font=("Helvetica", 11), bg="#e0e0ff", fg="black")
    
    def send_verbal_patch(self):
        if self.selected_marker_names:
            patch_msg = String(data=json.dumps({"reference": self.selected_marker_names}))
            self.pub_reference_patch.publish(patch_msg)
            self.selected_marker_names = []

            # Reset after sending
            self.selected_marker_ids.clear()
            self.selected_marker_names.clear()
            for btn, _ in self.marker_buttons:
                btn.config(relief="raised", font=("Helvetica", 11), bg="#e0e0ff", fg="black")

    def bind_ros(self):
        for section_topics in self.sections.values():
            for topic in section_topics:
                rospy.Subscriber(topic, String, self.update_msg(topic))

    def update_msg(self, topic):
        def callback(msg):
            self.latest_msgs[topic] = msg.data.strip()
            self.message_times[topic] = rospy.get_time()
        return callback

    def refresh_gui(self):
        now = rospy.get_time()
        for topic, widget in self.labels.items():
            content = self.latest_msgs.get(topic, "")
            t = self.message_times.get(topic, now)
            age = now - t

            if isinstance(widget, tk.Text):
                widget.config(state="normal")
                widget.delete("1.0", tk.END)
                widget.insert(tk.END, content)
                widget.config(state="disabled")
            else:
                # Age-based fading
                if age > 10:
                    color = "grey"
                elif topic in ["/listen_signal", "/execution_status"]:
                    color = "blue"
                elif topic in ["/wizard_intervene", "/error_log"]:
                    color = "red"
                else:
                    color = "black"
                widget.config(text=content, fg=color)

        self.root.after(200, self.refresh_gui)



def ros_thread():
    rospy.init_node("signal_coordinator_gui", anonymous=True, disable_signals=True)
    gui.bind_ros()
    rospy.spin()

def shutdown_all():
    print("\nShutting down processes...")
    for label, proc in process_refs.items():
        if proc and proc.poll() is None:  # still running
            print(f"Terminating {label}...")
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()

    # Kill head-mounted system
    print("Running kill_pro_head.bash...")
    try:
        subprocess.run(["bash", "kill_pro_head.sh"])
    except Exception as e:
        print(f"Error running kill_pro_head.sh: {e}")

def on_close():
    shutdown_all()
    root.destroy()

if __name__ == '__main__':
    root = tk.Tk()
    gui = SignalCoordinatorGUI(root)
    threading.Thread(target=ros_thread, daemon=True).start()
    # Bind window close
    root.protocol("WM_DELETE_WINDOW", on_close)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        on_close()
