import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import time

class FallDetectNoGUI(Node):
    def __init__(self):
        super().__init__('fall_detect_no_gui')
        # Create a subscriber to the 'camera/image_raw' topic
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # Topic name
            self.listener_callback,
            10  # QoS profile, you can adjust this
        )
        self.bridge = CvBridge()  # Bridge to convert ROS image messages to OpenCV
        self.pose_video = mp.solutions.pose.Pose(static_image_mode=False, min_detection_confidence=0.7, model_complexity=2)
        self.previous_avg_shoulder_height = 0
        self.time1 = time.time()
        
        self.get_logger().info("Fall detection node has been started.")

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Process the frame for pose detection and fall detection
        modified_frame, landmarks = self.detect_pose(frame)
        if landmarks is not None:
            fall_detected, self.previous_avg_shoulder_height = self.detect_fall(
                landmarks, frame.shape[0], self.previous_avg_shoulder_height
            )
            if fall_detected:
                self.get_logger().info("Fall detected!")

    def detect_pose(self, frame):
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose_video.process(frame_rgb)
        
        height, width, _ = frame.shape
        landmarks = []
        if results.pose_landmarks:
            for landmark in results.pose_landmarks.landmark:
                landmarks.append((int(landmark.x * width), int(landmark.y * height), (landmark.z * width)))
        else:
            return frame, None
        
        return frame, landmarks

    def detect_fall(self, landmarks, height, previous_avg_shoulder_height):
        left_shoulder_y = landmarks[11][1]
        right_shoulder_y = landmarks[12][1]
        
        # Calculate the average y-coordinate of the shoulder
        avg_shoulder_y = (left_shoulder_y + right_shoulder_y) / 2
        if previous_avg_shoulder_height == 0:
            previous_avg_shoulder_height = avg_shoulder_y
            return False, previous_avg_shoulder_height
        
        # Determine fall threshold and check if fall occurred
        fall_threshold = previous_avg_shoulder_height * 1.5
        if avg_shoulder_y > fall_threshold:
            previous_avg_shoulder_height = avg_shoulder_y
            return True, previous_avg_shoulder_height
        else:
            previous_avg_shoulder_height = avg_shoulder_y
            return False, previous_avg_shoulder_height


def main(args=None):
    rclpy.init(args=args)
    fall_detect_node = FallDetectNoGUI()
    
    try:
        rclpy.spin(fall_detect_node)
    except KeyboardInterrupt:
        pass
    finally:
        fall_detect_node.pose_video.close()
        fall_detect_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
