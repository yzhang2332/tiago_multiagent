from head_cam import HeadCamArucoDetector

detector = HeadCamArucoDetector()

# Start ROS spinning in a thread
import threading
threading.Thread(target=detector.spin, daemon=True).start()

# When ready, trigger the search
detector.search_for_marker(1)