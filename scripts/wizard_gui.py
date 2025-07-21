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
# config_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../config"))
# with open(os.path.join(config_dir, "object_aruco_high.json"), "r") as f:
#     aruco_data = json.load(f)
# with open(os.path.join(config_dir, "behavior_high.json"), "r") as f:
#     behavior_data = json.load(f)

def load_config_by_severity(severity):
    config_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../config"))
    if severity == "high":
        aruco_file = os.path.join(config_dir, "object_aruco_high.json")
        behavior_file = os.path.join(config_dir, "behavior_high.json")
    else:
        aruco_file = os.path.join(config_dir, "object_aruco_low.json")
        behavior_file = os.path.join(config_dir, "behavior_low.json")
    with open(aruco_file, "r") as f:
        aruco_data = json.load(f)
    with open(behavior_file, "r") as f:
        behavior_data = json.load(f)
    return aruco_data, behavior_data


# PRIMITIVES = list(behavior_data["primitives"].keys())
# META_ACTIONS = list(behavior_data["meta_actions"].keys())

# === Experiment Config Options and Commands ===
EXPERIMENT_CONFIG = {
    "Expliciteness": ["explicit", "implicit"],
    "Interpretation": ["correct", "incorrect"],
    "Severity": ["high", "low"]
}

COMMANDS = {
    "Mount pro head": ["bash", "mount_pro_head.sh"],
    "Wizard agent": ["python", "agent_wizard.py"],
    "Script and action agent": ["python", "agent_integrate.py"],
    "TTS service": ["python", "tts_service.py"],
    "Execution": ["python", "execute_control.py"],
    "Run pro head": ["bash", "run_pro_head.sh"],
    "Start script mode": ["python", "script_mode_interaction.py"],
    "Start ros1 tcp bridge": ["python", "ros1_tcp_bridge.py"],
    "Start phone display (app)": ["python", "phone_display/app.py"],
    "Start phone display (ros_client)": ["python", "phone_display/ros_client.py"],
    "Record rosbag": ["python", "record_rosbag.py"],
    "Speech interface": ["python", "speech_to_text.py"],
    "View head cam": ["python", "view_head_cam.py"]
}

REQUIRES_CONDITION_ARGS = {
    "agent_wizard.py",
    "agent_integrate.py",
    "script_mode_interaction.py"
}

thread_status = defaultdict(lambda: "inactive")
thread_refs = {}
process_refs = {}
status_labels = {}
selection = {
    "Expliciteness": None,
    "Interpretation": None,
    "Severity": None
}

def run_command(label, cmd):
    # If already running, terminate it
    if label in process_refs and process_refs[label] and process_refs[label].poll() is None:
        print(f"[GUI] Terminating active process: {label}")
        process_refs[label].terminate()
        try:
            process_refs[label].wait(timeout=5)
        except subprocess.TimeoutExpired:
            process_refs[label].kill()
        thread_status[label] = "inactive"
        update_status(label)
        return

    # Otherwise, start the process
    def target():
        thread_status[label] = "active"
        update_status(label)
        try:
            rospy.loginfo(f"[RUNNING] {label} → {' '.join(cmd)}")
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

def update_selected_config(config_display):
    config_display.config(
        text=f"Expliciteness: {selection['Expliciteness'] or '-'} | "
             f"Interpretation: {selection['Interpretation'] or '-'} | "
             f"Severity: {selection['Severity'] or '-'}"
    )

def build_experiment_column(root, gui):
    leftmost = tk.LabelFrame(root, text="Experiment Config", padx=8, pady=8, width=350)
    leftmost.pack(side="left", fill="y", padx=5, pady=5)
    leftmost.pack_propagate(False)

    config_display = tk.Label(leftmost, text="", wraplength=240, justify="left")
    config_display.pack(pady=10)
    update_selected_config(config_display)

    # Expliciteness
    tk.Label(leftmost, text="Select Expliciteness", font=("Helvetica", 11, "bold")).pack()
    for val in EXPERIMENT_CONFIG["Expliciteness"]:
        b = tk.Button(leftmost, text=val, width=20,
                      command=lambda v=val: gui.select_value("Expliciteness", v, config_display))
        b.pack()

    # Interpretation
    tk.Label(leftmost, text="Select Interpretation", font=("Helvetica", 11, "bold")).pack(pady=(10, 0))
    for val in EXPERIMENT_CONFIG["Interpretation"]:
        b = tk.Button(leftmost, text=val, width=20,
                      command=lambda v=val: gui.select_value("Interpretation", v, config_display))
        b.pack()

    # Severity
    tk.Label(leftmost, text="Select Severity", font=("Helvetica", 11, "bold")).pack(pady=(10, 0))
    for val in EXPERIMENT_CONFIG["Severity"]:
        b = tk.Button(leftmost, text=val, width=20,
                      command=lambda v=val: gui.select_value("Severity", v, config_display))
        b.pack()

    # Divider
    tk.Label(leftmost, text="---").pack(pady=8)

    tk.Label(leftmost, text="Run Processes", font=("Helvetica", 11, "bold")).pack()
    for label, cmd in COMMANDS.items():
        row = tk.Frame(leftmost)
        row.pack(fill="x", pady=2)

        def make_run_fn(label=label, cmd=cmd):
            def _run():
                script = cmd[1] if len(cmd) > 1 and cmd[0] == "python" else None

                if script in REQUIRES_CONDITION_ARGS:
                    if None in (selection["Expliciteness"], selection["Interpretation"], selection["Severity"]):
                        print(f"[GUI] Cannot launch '{label}' — missing experiment condition.")
                        return
                    args = [
                        "--expliciteness", selection["Expliciteness"],
                        "--interpretation", selection["Interpretation"],
                        "--severity", selection["Severity"]
                    ]
                    run_command(label, cmd + args)
                else:
                    run_command(label, cmd)
            return _run

        btn = tk.Button(row, text=label, width=25, anchor="w", command=make_run_fn())
        btn.pack(side="left")

        status = tk.Label(row, text=thread_status[label], fg="grey")
        status.pack(side="right")
        status_labels[label] = status

    return leftmost

class SignalCoordinatorGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Signal Coordinator Monitor")
        self.root.geometry("2000x1400")
        self.root.configure(bg="#f0f0f0")

        # ROS publishers
        self.pub_intervene = rospy.Publisher("/wizard_intervene", String, queue_size=1)
        self.pub_execute = rospy.Publisher("/wizard_agent/execute_sequence", String, queue_size=1)
        self.pub_patch = rospy.Publisher("/aruco_patch", String, queue_size=1)
        self.pub_reference_patch = rospy.Publisher("/reference_patch", String, queue_size=1)
        self.pub_listen = rospy.Publisher("/listen_signal", String, queue_size=1)
        self.pub_script_mode_status = rospy.Publisher("/script_mode_status", String, queue_size=1)
        self.pub_resume = rospy.Publisher("/script_manual_control", String, queue_size=1)
        self.pub_verbal = rospy.Publisher("/script_agent/verbal_response", String, queue_size=1)
        self.pub_remote_tts = rospy.Publisher("/ros1_to_tts", String, queue_size=1)


        # initialize primitives and markers
        self.aruco_data, self.behavior_data = load_config_by_severity("low")
        self.latest_msgs = {}
        self.labels = {}
        self.selected_primitives = []
        self.selected_marker_names = []
        self.selected_marker_ids = []

        self.sections = {
            "Wizard Agent Summary": ["/wizard_agent/summary"],
            "Speech and User Interaction": [
                "/listen_signal",
                "/received_utterance",
                "/script_agent/verbal_response",
                "/script_agent/action_instruction",
                "/tts_status"
            ],
            "System Interventions": [
                "/reference_patch",
                "/wizard_intervene",
                "/error_log"
            ],
            "Agent Status": [
                "/script_agent_status",
                "/action_agent_status",
                "/script_mode_status"
            ],
            "Robot Execution / Control": [
                "/action_agent/execute_sequence",
                "/wizard_agent/execute_sequence",
                "/execution_status"
            ]
        }

        self.message_times = {}

        # === Layout: Left | Center | Right ===
        main_frame = tk.Frame(self.root)
        main_frame.pack(fill="both", expand=True)

        self.exp_column = build_experiment_column(main_frame, self)

        self.left_frame = tk.Frame(main_frame, width=700)
        self.left_frame.pack(side="left", fill="both", expand=True)
        self.left_frame.pack_propagate(False)

        # Container for right side (split into Wizard + TTS)
        self.right_container = tk.Frame(main_frame)
        self.right_container.pack(side="right", fill="y", padx=10, pady=10)

        # Wizard controls in the left half of right side
        self.wizard_frame = tk.LabelFrame(self.right_container, text="Wizard Controls", font=("Helvetica", 14, "bold"), padx=8, pady=8, width=600)
        self.wizard_frame.pack(side="left", fill="y", padx=(0, 10))
        self.wizard_frame.pack_propagate(False)

        # TTS buttons in the right half of right side
        self.tts_frame_container = tk.LabelFrame(self.right_container, text="All TTS Buttons", font=("Helvetica", 14, "bold"), padx=8, pady=8, width=300)
        self.tts_frame_container.pack(side="right", fill="y")
        self.tts_frame_container.pack_propagate(False)


        for section_title, topics in self.sections.items():
            self.add_section(section_title, topics)

        self.add_wizard_controls()
        self.root.after(200, self.refresh_gui)

    def update_action_marker_buttons(self):
        # Remove existing buttons
        for btn, _ in getattr(self, "primitive_buttons", []):
            btn.destroy()
        for btn, _ in getattr(self, "marker_buttons", []):
            btn.destroy()
        self.primitive_buttons = []
        self.marker_buttons = []

        PRIMITIVES = list(self.behavior_data["primitives"].keys())
        META_ACTIONS = list(self.behavior_data["meta_actions"].keys())

        action_frame = self.action_frame
        marker_frame = self.marker_frame

        # Actions
        for child in action_frame.winfo_children():
            child.destroy()
        for name in META_ACTIONS + PRIMITIVES:
            btn = tk.Button(action_frame, text=name,
                            command=lambda n=name: self.toggle_primitive(n),
                            font=("Helvetica", 11), bg="#ddffee", relief="raised")
            btn.pack(fill="x", pady=1)
            self.primitive_buttons.append((btn, name))

        # Markers
        for child in marker_frame.winfo_children():
            child.destroy()
        for marker in self.aruco_data["markers"]:
            mid = marker["id"]
            name = marker["name"]
            btn = tk.Button(marker_frame, text=f"{name} (ID: {mid})",
                            command=lambda m=mid: self.select_marker(m),
                            wraplength=240, font=("Helvetica", 11),
                            bg="#e0e0ff", relief="raised")
            btn.pack(fill="x", pady=1)
            self.marker_buttons.append((btn, mid))
    
    def update_scripted_tts_buttons(self):
        for widget in self.script_tts_frame.winfo_children():
            widget.destroy()

        exp = selection.get("Expliciteness") or "explicit"
        interp = selection.get("Interpretation") or "correct"
        severity = selection.get("Severity") or "high"
        condition_key = f"{severity}_{exp}_{interp}"
        script_lines = self.script_map.get(condition_key, [])

        instructor_idx = 0
        tiago_idx = 0

        for role, text in script_lines:
            if role == "instructor":
                btn = tk.Button(self.script_tts_frame, text=f"Instructor: {text[-3:]}", font=("Helvetica", 11),
                                command=lambda t=text: self.send_remote_tts_phrase(t),
                                bg="#e6f7ff", relief="raised", anchor="w", width=10)
                btn.grid(row=instructor_idx, column=0, padx=2, pady=2, sticky="ew")
                instructor_idx += 1
            elif role == "tiago":
                btn = tk.Button(self.script_tts_frame, text=f"Tiago: {text[:4]}", font=("Helvetica", 11),
                                command=lambda t=text: self.send_tts_phrase(t),
                                bg="#fffbe6", relief="raised", anchor="w", width=10)
                btn.grid(row=tiago_idx, column=1, padx=2, pady=2, sticky="ew")
                tiago_idx += 1

    def select_value(self, key, value, config_display):
        selection[key] = value
        update_selected_config(config_display)
        # Reload action and marker buttons based on new severity
        if key == "Severity":
            self.aruco_data, self.behavior_data = load_config_by_severity(value)
            self.update_action_marker_buttons()
        
        self.update_scripted_tts_buttons()

    
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
                    label_frame, height=12, wrap="word", font=font,
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
        # --- Start/Stop Listening Buttons ---
        listen_row = tk.Frame(self.wizard_frame)
        listen_row.pack(pady=(4, 4), fill="x")

        tk.Button(
            listen_row, text="Start Listening", bg="#5bc0de", fg="white",
            font=("Helvetica", 12, "bold"), command=self.send_start_listening,
            width=18
        ).pack(side="left", padx=5, fill="x", expand=True)

        tk.Button(
            listen_row, text="Stop Listening", bg="#777777", fg="white",
            font=("Helvetica", 12, "bold"), command=self.send_stop_listening,
            width=18
        ).pack(side="left", padx=5, fill="x", expand=True)

        # === Grouped Action + Marker Selection Side by Side ===
        action_marker_frame = tk.Frame(self.wizard_frame)
        action_marker_frame.pack(pady=10, fill="x")

        # --- Actions ---
        action_frame = tk.LabelFrame(action_marker_frame, text="Select Action", font=("Helvetica", 12, "bold"), padx=4, pady=4)
        action_frame.pack(side="left", fill="both", expand=True, padx=5)

        self.primitive_buttons = []
        PRIMITIVES = list(self.behavior_data["primitives"].keys())
        META_ACTIONS = list(self.behavior_data["meta_actions"].keys())
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
        for marker in self.aruco_data["markers"]:
            mid = marker["id"]
            name = marker["name"]
            btn = tk.Button(marker_frame, text=f"{name} (ID: {mid})",
                            command=lambda m=mid: self.select_marker(m),
                            wraplength=240, font=("Helvetica", 11),
                            bg="#e0e0ff", relief="raised")
            btn.pack(fill="x", pady=1)
            self.marker_buttons.append((btn, mid))

        self.action_frame = action_frame
        self.marker_frame = marker_frame

        # --- Takeover and Stop Takeover Buttons Side by Side ---
        takeover_row = tk.Frame(self.wizard_frame)
        takeover_row.pack(pady=(0, 4), fill="x")
        takeover_row.grid_columnconfigure(0, weight=1, uniform="group1")
        takeover_row.grid_columnconfigure(1, weight=1, uniform="group1")

        tk.Button(
            takeover_row, text="Takeover", bg="#d9534f", fg="white",
            font=("Helvetica", 12, "bold"), command=self.send_takeover,
        ).grid(row=0, column=0, padx=5, sticky="ew")

        tk.Button(
            takeover_row, text="Stop Takeover", bg="#777777", fg="white",
            font=("Helvetica", 12, "bold"), command=self.send_stop_takeover,
        ).grid(row=0, column=1, padx=5, sticky="ew")

        # --- Update & Verbal Patch Buttons Side by Side ---
        button_row = tk.Frame(self.wizard_frame)
        button_row.pack(pady=(0, 12), fill="x")

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

        # --- Start Head Button ---
        start_row = tk.Frame(self.wizard_frame)
        start_row.pack(pady=(4, 4), fill="x")

        tk.Button(
            start_row, text="Start Head", bg="#d9534f", fg="white",
            font=("Helvetica", 12, "bold"), command=self.send_start_head
        ).pack(fill="x", expand=True, padx=5)

        # --- Resume and Pause Progress Buttons Side by Side ---
        progress_row = tk.Frame(self.wizard_frame)
        progress_row.pack(pady=(0, 4), fill="x")
        progress_row.grid_columnconfigure(0, weight=1, uniform="group2")
        progress_row.grid_columnconfigure(1, weight=1, uniform="group2")

        tk.Button(
            progress_row, text="Resume Progress", bg="#5bc0de", fg="white",
            font=("Helvetica", 12, "bold"), command=lambda: self.pub_resume.publish("resume"),
        ).grid(row=0, column=0, padx=5, sticky="ew")

        tk.Button(
            progress_row, text="Pause Progress", bg="#777777", fg="white",
            font=("Helvetica", 12, "bold"), command=lambda: self.pub_resume.publish("pause"),
        ).grid(row=0, column=1, padx=5, sticky="ew")

        # --- TTS Buttons ---
        tts_frame = tk.LabelFrame(self.tts_frame_container, text="TTS", font=("Helvetica", 12, "bold"), padx=4, pady=4)
        tts_frame.pack(fill="x", padx=5, pady=8)

        tts_phrases = [
            ("Tiago Intro", "Hi, I’m Tiago. I’ll be here with you throughout the task. Looking forward to working with you."),
            ("Let's start", "Shall we start? How can I help you?"),
            ("Moment", "Give me a moment."),
            ("Think", "Let me think."),
            ("Work", "Working on it."),
            ("Done", "Done."),
            ("Next", "What should I do next?"),
            ("Thank", "Thank you.")
        ]

        remote_tts_phrases = [
            ("Diego Intro", "introduction")
        ]

        # 两列布局
        max_len = max(len(tts_phrases), len(remote_tts_phrases))
        for i in range(max_len):
            # Tiago TTS
            if i < len(tts_phrases):
                btn_text, tts_text = tts_phrases[i]
                btn = tk.Button(tts_frame, text=btn_text, font=("Helvetica", 11),
                                command=lambda t=tts_text: self.send_tts_phrase(t),
                                bg="#fffbe6", relief="raised", anchor="w", width=10)
                btn.grid(row=i, column=0, padx=2, pady=2, sticky="ew")
            # Diego TTS
            if i < len(remote_tts_phrases):
                btn_text, tts_text = remote_tts_phrases[i]
                btn = tk.Button(tts_frame, text=btn_text, font=("Helvetica", 11),
                                command=lambda t=tts_text: self.send_remote_tts_phrase(t),
                                bg="#e6f7ff", relief="raised", anchor="w", width=10)
                btn.grid(row=i, column=1, padx=2, pady=2, sticky="ew")

        self.script_map = {
            "high_explicit_correct": [
                ("instructor", "high_explicit_correct_1_1"),
                ("tiago", "Do you want me to hold the test tube for you?"),
                ("instructor", "high_explicit_correct_1_2"),
                ("tiago", "Sure. I will hold the test tube for you."),
                ("instructor", "high_explicit_correct_2_1"),
                ("tiago", "Do you want me to shake the test tube?"),
                ("instructor", "high_explicit_correct_2_2"),
                ("tiago", "Got it. I'll shake the test tube."),
                ("instructor", "high_explicit_correct_3_1"),
                ("tiago", "Do you want me to shake the test tube again?"),
                ("instructor", "high_explicit_correct_3_2"),
                ("tiago", "Sure, I will shake the test tube again")
            ],
            "high_implicit_correct": [
                ("instructor", "high_implicit_correct_1_1"),
                ("tiago", "Sure. I will hold the test tube for you."),
                ("instructor", "high_implicit_correct_2_1"),
                ("tiago", "Got it. I'll shake the test tube."),
                ("instructor", "high_implicit_correct_3_1"),
                ("tiago", "Sure, I will shake the test tube again")
            ],
            "high_explicit_incorrect": [
                ("instructor", "high_explicit_incorrect_1_1"),
                ("tiago", "Do you want me to hold your arm when you pour the powder?"),
                ("instructor", "high_explicit_incorrect_1_2"),
                ("tiago", "Sure. I will hold the test tube for you."),
                ("instructor", "high_explicit_incorrect_2_1"),
                ("tiago", " Did we do it wrong? Should we start over?"),
                ("instructor", "high_explicit_incorrect_2_2"),
                ("tiago", "Got it. I'll shake the test tube."),
                ("instructor", "high_explicit_incorrect_3_1"),
                ("tiago", "Want to add another 30 grams of powder"),
                ("instructor", "high_explicit_incorrect_3_2"),
                ("tiago", "Sure, I will shake the test tube again")
            ],
            "high_implicit_incorrect": [
                ("instructor", "high_implicit_incorrect_1_1"),
                ("tiago", "Okay, I'll hold your arm when you pour the powder."),
                ("instructor", "high_implicit_incorrect_1_2"),
                ("tiago", "Sure. I will hold the test tube for you."),
                ("instructor", "high_implicit_incorrect_2_1"),
                ("tiago", " Sounds like we did it wrong. Let's start over."),
                ("instructor", "high_implicit_incorrect_2_2"),
                ("tiago", "Got it. I'll shake the test tube."),
                ("instructor", "high_implicit_incorrect_3_1"),
                ("tiago", "Sure, I'll add another 30 grams of powder"),
                ("instructor", "high_implicit_incorrect_3_2"),
                ("tiago", "Sure, I will shake the test tube again")
            ],
            "low_explicit_correct": [
                ("instructor", "low_explicit_correct_1_1"),
                ("tiago", "Do you want me to put the classical work, Triumph of Galatea, to the top left position?"),
                ("instructor", "low_explicit_correct_1_2"),
                ("tiago", "Sure. I will put the Triumph of Galatea to the top left position."),
                ("instructor", "low_explicit_correct_2_1"),
                ("tiago", "Do you want me to put the peaceful work, Impression Sunrise, to the top middle position?"),
                ("instructor", "low_explicit_correct_2_2"),
                ("tiago", "Got it. I'll put the Impression Sunrise to the top middle position."),
                ("instructor", "low_explicit_correct_3_1"),
                ("tiago", "Do you want me to put the one with soft watch, Persistence of Memory, to the top right position?"),
                ("instructor", "low_explicit_correct_3_2"),
                ("tiago", "Sure, I will put the Persistence of Memory to the top right position.")
            ],
            "low_implicit_correct": [
                ("instructor", "low_implicit_correct_1_1"),
                ("tiago", "Sure. I will put the classical work, Triumph of Galatea, to the top left position."),
                ("instructor", "low_implicit_correct_2_1"),
                ("tiago", "Got it. I'll put the peaceful work, Impression Sunrise, to the top middle position."),
                ("instructor", "low_implicit_correct_3_1"),
                ("tiago", "Sure, I will put the one with soft watch, Persistence of Memory, to the top right position.")
            ],
            # "low_explicit_incorrect": [
            #     ("instructor", "low_explicit_incorrect_1_1"),
            #     ("tiago", "Do you want me to put the classical work, Triumph of Galatea, to the bottom middle position?"),
            #     ("instructor", "low_explicit_incorrect_1_2"),
            #     ("tiago", "Sure. I will put the Triumph of Galatea to the top left position."),
            #     ("instructor", "low_explicit_incorrect_2_1"),
            #     ("tiago", "Do you want me to replace the tragic one, Raft of Medusa, with the peaceful one, Impression Sunrise, in the bottom middle position?"),
            #     ("instructor", "low_explicit_incorrect_2_2"),
            #     ("tiago", "Got it. I'll put the Impression Sunrise to the top middle position."),
            #     ("instructor", "low_explicit_incorrect_3_1"),
            #     ("tiago", "Do you want me to put another Impression Sunrise to the top right position?"),
            #     ("instructor", "low_explicit_incorrect_3_2"),
            #     ("tiago", "Sure, I will put the one with soft watch, Persistence of Memory, to the top right position.")
            # ],
            "low_explicit_incorrect": [
                ("instructor", "low_explicit_incorrect_1_1"),
                ("tiago", "Do you want me to put popcorn near the exit?"),
                ("instructor", "low_explicit_incorrect_1_2"),
                ("tiago", "Sure. I will put durians to the top right position."),
                ("instructor", "low_explicit_incorrect_2_1"),
                ("tiago", "Do you want me to replace pineapples with potatos in the bottom right position?"),
                ("instructor", "low_explicit_incorrect_2_2"),
                ("tiago", "Got it. I'll put potatos to the top middle position."),
                ("instructor", "low_explicit_incorrect_3_1"),
                ("tiago", "Do you want me to put the last item, popcorn, also to the top middle position?"),
                ("instructor", "low_explicit_incorrect_3_2"),
                ("tiago", "Sure, I will put the popcorn to the top left position.")
            ],
            "low_implicit_incorrect": [
                ("instructor", "low_implicit_incorrect_1_1"),
                ("tiago", "Okay, I'll put the classical work, Triumph of Galatea, to the bottom middle position."),
                ("instructor", "low_implicit_incorrect_1_2"),
                ("tiago", "Sure. I will put the Triumph of Galatea to the top left position."),
                ("instructor", "low_implicit_incorrect_2_1"),
                ("tiago", " Sounds like I'm going to replace the tragic one, Raft of Medusa, with the peaceful one, Impression Sunrise."),
                ("instructor", "low_implicit_incorrect_2_2"),
                ("tiago", "Got it. I'll put the Impression Sunrise to the top middle position."),
                ("instructor", "low_implicit_incorrect_3_1"),
                ("tiago", "I can't put another Impression Sunrise to the top right position. There is only one."),
                ("instructor", "low_implicit_incorrect_3_2"),
                ("tiago", "Sure, I will put the one with soft watch, Persistence of Memory, to the top right position.")
            ]
        }

        self.script_tts_frame = tk.LabelFrame(self.tts_frame_container, text="Scripted TTS", font=("Helvetica", 12, "bold"), padx=4, pady=4)
        self.script_tts_frame.pack(fill="x", padx=5, pady=8)

        self.update_scripted_tts_buttons()

    def send_tts_phrase(self, text):
        self.pub_verbal.publish(text)

    def send_remote_tts_phrase(self, text):
        self.pub_remote_tts.publish(text)
    
    def send_start_head(self):
        self.pub_script_mode_status.publish("start_head")

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
        # if marker_id not in self.selected_marker_ids:
        #     self.selected_marker_ids.append(marker_id)

        #     name = next((marker["name"] for marker in self.aruco_data["markers"] if marker["id"] == marker_id), str(marker_id))
        #     self.selected_marker_names.append(name)
        if marker_id in self.selected_marker_ids:
            # Deselect marker
            index = self.selected_marker_ids.index(marker_id)
            self.selected_marker_ids.pop(index)
            self.selected_marker_names.pop(index)
        else:
            # Select marker
            self.selected_marker_ids.append(marker_id)
            name = next((marker["name"] for marker in self.aruco_data["markers"] if marker["id"] == marker_id), str(marker_id))
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

    def send_start_listening(self):
        self.pub_intervene.publish("takeover")
        rospy.sleep(0.1)
        self.pub_intervene.publish("stop_takeover")
        rospy.sleep(1.5)
        self.pub_listen.publish("start_listen")

    def send_stop_listening(self):
        self.pub_listen.publish("stop_listen")

    def send_takeover(self):
        self.pub_intervene.publish("takeover")
    
    def send_stop_takeover(self):
        # pub = rospy.Publisher("/execution_status", String, queue_size=1)
        # rospy.sleep(0.1)
        self.pub_intervene.publish("stop_takeover")
        # pub.publish("finished")

    def update_and_takeover(self):
        self.send_takeover()
        rospy.sleep(0.1)  # Ensure the intervene message is processed first
        if self.selected_primitives:
            plan = []
            last_marker_id = self.selected_marker_ids[-1] if self.selected_marker_ids else None
            for p in self.selected_primitives:
                if p in self.behavior_data["meta_actions"]:
                    sequence = self.behavior_data["meta_actions"][p]["sequence"]
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

            if age > 10:
                color = "grey"
            elif topic in ["/listen_signal", "/execution_status"]:
                color = "blue"
            elif topic in ["/wizard_intervene", "/error_log"]:
                color = "red"
            else:
                color = "black"

            if isinstance(widget, tk.Text):
                widget.config(state="normal")
                widget.delete("1.0", tk.END)
                widget.insert(tk.END, content)
                widget.config(fg=color)
                widget.config(state="disabled")
            else:
                # # Age-based fading
                # if age > 10:
                #     color = "grey"
                # elif topic in ["/listen_signal", "/execution_status"]:
                #     color = "blue"
                # elif topic in ["/wizard_intervene", "/error_log"]:
                #     color = "red"
                # else:
                #     color = "black"
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
