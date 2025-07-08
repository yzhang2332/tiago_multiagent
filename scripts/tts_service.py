import rospy
from std_msgs.msg import String
from pal_interaction_msgs.msg import TtsAction, TtsGoal
import actionlib
import re
import threading
import json

class TTSService():
    def __init__(self):
        rospy.init_node("tts_service_node", anonymous=True)

        # Set up the TTS action client
        self.tts_client = actionlib.SimpleActionClient('/tts', TtsAction)
        rospy.loginfo("Waiting for TTS action server...")
        self.tts_client.wait_for_server()
        rospy.loginfo("TTS server connected.")

        # Subscribe to the topic that provides text to speak
        rospy.Subscriber("/script_agent/verbal_response", String, self.tts_callback)

        # Publisher to TTS status
        self.tts_status_pub = rospy.Publisher("/tts_status", String, queue_size=10)
        self.require_patch_pub = rospy.Publisher("/reference_patch", String, queue_size=10)

        # Placeholder for patch data
        self.pending_patches = []
        self.patch_event = threading.Event()

        # Start patch subscriber (but it waits only when needed)
        rospy.Subscriber("/reference_patch", String, self.patch_callback)

        rospy.loginfo("TTSService ready and listening for verbal responses.")
        self.tts_status_pub.publish("waiting")
        rospy.spin()

    def tts(self, text):
        # Create a goal to say the given sentence
        goal = TtsGoal()
        goal.rawtext.text = text
        goal.rawtext.lang_id = "en_GB"  # Use British English
        self.tts_client.send_goal_and_wait(goal)

        rospy.sleep(0.2)

        # After TTS is done, publish "finished"
        self.tts_status_pub.publish("finished")
        rospy.sleep(1.0)
        self.tts_status_pub.publish("waiting")
    
    def patch_callback(self, msg):
        try:
            data = json.loads(msg.data)
            references = data.get("reference", [])
            if isinstance(references, list):
                self.reference_patch = references
                self.patch_event.set()
            else:
                rospy.logwarn("Reference patch malformed — 'reference' must be a list.")
        except Exception as e:
            rospy.logwarn(f"Invalid JSON on /reference_patch: {e}")

    def wait_for_patches(self, expected_count):
        self.pending_patches = []
        self.patch_event.clear()

        timeout = rospy.Time.now() + rospy.Duration(10.0)  # wait max 10 sec
        while rospy.Time.now() < timeout:
            if self.patch_event.wait(timeout=0.1) and self.reference_patch is not None:
                if len(self.reference_patch) == expected_count:
                    return self.reference_patch
                else:
                    rospy.logwarn("Reference patch count mismatch.")
                    return None
        return None
    
    def tts_callback(self, msg):
        raw_text = msg.data
        self.tts_status_pub.publish("received")
        rospy.loginfo(f"Received text for TTS: {raw_text}")

        placeholders = re.findall(r"<wizard_input>", raw_text)
        count = len(placeholders)

        if count == 0:
            self.tts(raw_text)
        else:
            self.require_patch_pub.publish("require_patch")
            rospy.loginfo(f"Detected {count} <wizard_input> placeholder(s), waiting for reference_patch input...")
            patches = self.wait_for_patches(count)

            if not patches:
                rospy.logwarn("No valid patches received in time. Skipping TTS.")
                self.tts_status_pub.publish("failed")
                return

            for patch in patches:
                raw_text = raw_text.replace("<wizard_input>", patch, 1)

            rospy.loginfo(f"Resolved text: {raw_text}")
            self.tts(raw_text)
            self.require_patch_pub.publish("finished")

if __name__ == '__main__':
    try:
        TTSService()
    except rospy.ROSInterruptException:
        pass
