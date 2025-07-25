#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import openai
import tempfile
import soundfile as sf
import sounddevice as sd
import numpy as np
import sys
import os
from nltk.tokenize import word_tokenize

# Configure your OpenAI API key here
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from utils.load_config import load_openai_config

# Load config
cfg = load_openai_config()
openai.api_key = cfg["openai_api_key"]

# TODO: initial flask app: leave for later 
# run_app_in_thread()


class VoiceRecognitionServer:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('voice_recognition', anonymous=True)
        
        # Create a publisher for the recognized text
        self.text_pub = rospy.Publisher('/received_utterance', String, queue_size=10)
        self.listen_pub = rospy.Publisher("/listen_signal", String, queue_size=10)
        self.listen_signal_pub = rospy.Publisher("/start_listen_signal", String, queue_size=10)

        # Audio recording parameters
        self.sample_rate = 16000 # 16000 44100 48000
        self.threshold = 3.5 # SilencTruee detection threshold
        self.silence_duration = 3  # Seconds of silence to consider the speaker has stopped
        self.stream = None
        self.device_index = None

        self.listen_flag = False
        self.listen_subscriber = rospy.Subscriber("/listen_signal", String, self.listen_callback)
        
    
    # def listen_callback(self, msg):
    #     rospy.loginfo(f"Received listen signal: {msg.data}")
    #     if msg.data.lower() == "start_listen":
    #         self.listen_flag = True

    def listen_callback(self, msg):
        rospy.loginfo(f"Received listen signal: {msg.data}")
        if msg.data.lower() == "start_listen":
            self.listen_flag = True
        elif msg.data.lower() == "stop_listen":
            self.listen_flag = False


    def calibrate_threshold(self, calibration_duration=1):
        """Automatically calibrate the noise threshold."""
        rospy.loginfo("Calibrating microphone. Please remain silent...")
        recording = sd.rec(int(calibration_duration * self.sample_rate), 
                           samplerate=self.sample_rate, channels=1, 
                           device=self.device_index, dtype='float32')
        sd.wait()  # Wait for the recording to finish
        # Calculate the RMS of the recording
        rms = np.sqrt(np.mean(np.square(recording), axis=0))
        if np.max(rms) > 0.03:
            self.threshold = np.max(rms) * 100
        else:
            self.threshold = 3.5  # set minimum threshold
        rospy.loginfo(f"Calibration complete. New threshold: {self.threshold}")

    # def record_until_silence(self):
    #     """Record from the microphone until silence is detected."""
    #     rospy.loginfo("Starting recording...")
    #     recorded_data = []
    #     silent_frames = 0
    #     recording = False

    #     def callback(indata, frames, time, status):
    #         nonlocal recorded_data, silent_frames, recording #, silent_frames_buffer
    #         if status:
    #             print(status, file=sys.stderr)
    #         amplitude = np.linalg.norm(indata)*9

    #         if amplitude < self.threshold:
    #             if recording:
    #                 silent_frames += 1
    #             if silent_frames > self.sample_rate / frames * self.silence_duration:
    #                 raise sd.CallbackStop
    #         else:
    #             recording = True
    #             silent_frames = 0
                
    #             recorded_data.append(indata.copy())
    #             # print(recorded_data[-1])
        
    #     with sd.InputStream(callback=callback, samplerate=self.sample_rate, 
    #                         channels=1, device=self.device_index, dtype='float32'):
    #         try:
    #             sd.sleep(4000)  # Failsafe timeout
    #         except sd.CallbackStop:
    #             pass
        
    #     rospy.loginfo("Recording stopped.")
        
    #     if recorded_data:
    #         return np.concatenate(recorded_data, axis=0)
    #     else:
    #         rospy.logwarn("No audio recorded – returning empty array.")
    #         return np.array([], dtype='float32')

    def record_until_silence(self):
        rospy.loginfo("Starting recording...")
        recorded_data = []
        silent_frames = 0
        recording = False

        min_duration_seconds = 3
        min_frames = int(min_duration_seconds * self.sample_rate)

        def callback(indata, frames, time, status):
            nonlocal recorded_data, silent_frames, recording
            if status:
                print(status, file=sys.stderr)

            amplitude = np.linalg.norm(indata) * 3
            max_silent_frames = int(self.sample_rate * self.silence_duration / frames)

            if amplitude < self.threshold:
                if recording and len(recorded_data) * frames > min_frames:
                    silent_frames += 1
            else:
                silent_frames = 0
                if not recording:
                    recording = True
                    rospy.loginfo("Voice detected, start recording.")

            if recording:
                recorded_data.append(indata.copy())

            if silent_frames > max_silent_frames:
                raise sd.CallbackStop

        with sd.InputStream(callback=callback, samplerate=self.sample_rate,
                            channels=1, device=self.device_index, dtype='float32'):
            try:
                sd.sleep(10000)  # generous timeout
            except sd.CallbackStop:
                pass

        rospy.loginfo("Recording stopped.")

        if recorded_data:
            audio = np.concatenate(recorded_data, axis=0)
            rospy.loginfo(f"Recorded {audio.shape[0]} samples.")
            return audio
        else:
            rospy.logwarn("No audio recorded – returning empty array.")
            return np.array([], dtype='float32')

    
    
    def recognize_speech(self):    

        # Record audio until silence
        audio_data = self.record_until_silence()
        if audio_data.size == 0:
            rospy.logwarn("Audio data is empty. Skipping transcription.")
            return None

        self.listen_pub.publish("stop_listen")
        
        # Save audio data to a temporary file
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=True) as tmpfile:
            sf.write(tmpfile, audio_data, self.sample_rate)
            
            # Open the temporary file for reading
            try:
                with open(tmpfile.name, 'rb') as audio_file:
                    # Transcribe audio file using OpenAI's model
                    transcript = openai.audio.transcriptions.create(
                        model="gpt-4o-transcribe",
                        file=audio_file,
                        response_format = "text",
                        language="en"
                        # can also add prompt here
                    )
                    # Extract the transcript text
                    text = transcript.strip()
                    return text
            except openai.OpenAIError as e:
                rospy.logwarn(f"OpenAI error: {e}")
            except Exception as e:
                rospy.logwarn(f"Unexpected error: {e}")
        return None


    def processing(self):
        text = self.recognize_speech()
        if text:
            self.listen_flag = False
            self.text_pub.publish(text)
        else:
            rospy.loginfo("No text recognized or error occurred.")
            if self.listen_flag:
                rospy.loginfo("listen_flag is True, restarting listening.")
                self.listen_pub.publish("start_listen")
            else:
                rospy.loginfo("listen_flag is False, stopping listening.")
                self.listen_pub.publish("stop_listen")
    

    def get_mic_array_index(self) -> int:
        dev_list = sd.query_devices()  # This is a list of dictionaries
        for dev in dev_list:
            print(dev['name'])
            if "Mic Array" in dev['name']:
                return dev_list.index(dev)
        return -1
                

    def run(self):        
        dev_id = self.get_mic_array_index()
        if dev_id == -1:
            rospy.logerr("Microphone not found!")
            rospy.signal_shutdown("Microphone not found")
        else:
            rospy.loginfo(f"Device found with ID: {dev_id}")
            self.device_index = dev_id


        # Calibrate threshold before recording
        self.calibrate_threshold()
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.listen_flag:
                rospy.loginfo("Listening triggered.")
                self.listen_signal_pub.publish("ka")
                rospy.sleep(0.5)
                
                self.processing()
            rate.sleep()
            

if __name__ == '__main__':
    vr_server = VoiceRecognitionServer()
    try:
        vr_server.run()
    except rospy.ROSInterruptException:
        pass