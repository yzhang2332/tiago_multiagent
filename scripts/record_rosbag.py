# import subprocess
# import datetime
# import os
# import time

# # Ensure the directory exists
# # directory = "/home/pal/rosbags/"
# directory = "/home/yanzhang/pal/study2/"
# if not os.path.exists(directory):
#     os.makedirs(directory)
    

# # Format the current date and time as a string
# current_time = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")


# # Define the bag file name using the current date and time
# # bag_file_name = f"/home/rosbags/record_{current_time}.bag"
# bag_file_name = f"{directory}record_{current_time}.bag"

# # Throttle command for the camera topic
# throttle_rgb_command = "rosrun topic_tools throttle messages /xtion/rgb/image_raw 6.0 /xtion/rgb/image_raw_throttled"
# throttle_rgb_process = subprocess.Popen(throttle_rgb_command, shell=True, preexec_fn=os.setsid)

# throttle_dep_command = "rosrun topic_tools throttle messages /xtion/depth_registered/image_raw 6.0 /xtion/depth/image_raw_throttled"

# throttle_dep_process = subprocess.Popen(throttle_dep_command, shell=True, preexec_fn=os.setsid)

# # Adjusted topics with throttled camera topic
# topics = "/xtion/rgb/image_raw_throttled /xtion/depth/image_raw_throttled /joint_states /script_mode_status /listen_signal /script_agent/action_instruction /tts_status /execution_status /script_agent_status /action_agent_status /wizard_intervene /wizard_agent/summary /received_utterance /script_agent/verbal_response /reference_patch /gripper_cam_aruco_pose /action_agent/execute_sequence /wizard_agent/execute_sequence /aruco_patch /arm_controller/command /error_log" 

# # Define the command to start recording all topics to the named bag file
# command = f"rosbag record --lz4 {topics} -O {bag_file_name}"
# # Start recording
# process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)

# # Keep the script running until Ctrl+C is pressed
# print("Recording... Press Ctrl+C to stop.")
# while True:
#     time.sleep(1)