import rospy
from std_msgs.msg import String
from pal_interaction_msgs.msg import TtsAction, TtsGoal
import actionlib

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

        rospy.loginfo("TTSService ready and listening for verbal responses.")
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

    def tts_callback(self, msg):
        rospy.loginfo(f"Received text for TTS: {msg.data}")
        self.tts_status_pub.publish("received")
        self.tts(msg.data)

if __name__ == '__main__':
    try:
        TTSService()
    except rospy.ROSInterruptException:
        pass
