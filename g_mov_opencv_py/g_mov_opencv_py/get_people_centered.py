import cv2
import mediapipe as mp
import time

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist  # Message type for velocity commands

class FallDetectNoGUI(Node):
    def __init__(self):
        super().__init__('fall_detect_no_gui')
        
        # Subscriber to the camera image topic
        self.img_subs_ = self.create_subscription(
            Image, '/image_raw', self.listener_callback, 10)
        
        # Publisher for fall detection
        self.fall_pub_ = self.create_publisher(Bool, '/fall_detect', 10)
        
        # Publisher for velocity commands
        self.cmd_vel_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.bridge = CvBridge()
        self.pose_video = mp.solutions.pose.Pose(
            static_image_mode=False, min_detection_confidence=0.7, model_complexity=2)
        
        self.previous_avg_shoulder_height = 0
        self.time1 = time.time()
        
        self.get_logger().info("Fall detection (NO GUI) node has been started.")

    def listener_callback(self, msg):
        flag_fall = Bool()
        flag_fall.data = False
        
        # Convert the incoming image to an OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Process the frame to detect pose and landmarks
        modified_frame, landmarks = self.detect_pose(frame)
        if landmarks is not None:
            # Fall detection logic
            fall_detected_, self.previous_avg_shoulder_height = self.detect_fall(
                landmarks, frame.shape[0], self.previous_avg_shoulder_height)
            if fall_detected_:
                self.get_logger().info("Fall detected!")
                flag_fall.data = True
            self.fall_pub_.publish(flag_fall)
            
            # Person tracking logic to keep them in the middle of the frame
            self.track_person(landmarks, frame.shape[1])
        else:
            # Stop movement if no person is detected
            stop_twist = Twist()
            self.cmd_vel_pub_.publish(stop_twist)

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
        avg_shoulder_y = (left_shoulder_y + right_shoulder_y) / 2
        
        if previous_avg_shoulder_height == 0:
            previous_avg_shoulder_height = avg_shoulder_y
            return False, previous_avg_shoulder_height
        
        fall_threshold = previous_avg_shoulder_height * 1.5
        if avg_shoulder_y > fall_threshold:
            previous_avg_shoulder_height = avg_shoulder_y
            return True, previous_avg_shoulder_height
        else:
            previous_avg_shoulder_height = avg_shoulder_y
            return False, previous_avg_shoulder_height

    def track_person(self, landmarks, frame_width):
        # Calculate x-position of the nose landmark (index 0)
        nose_x = landmarks[0][0]
        center_x = frame_width // 2  # Center of the frame

        # Determine the error between nose position and center of the frame
        error = nose_x - center_x
        
        # Initialize a Twist message to adjust angular velocity
        twist_msg = Twist()
        
        # Define a proportional gain for angular velocity adjustment
        k_angular = 0.002
        
        # If the error is outside a tolerance, set angular velocity to turn the robot
        if abs(error) > 30:  # Tolerance in pixels
            twist_msg.angular.z = -k_angular * error
        else:
            twist_msg.angular.z = 0.0  # Stop rotation if within tolerance
        
        # Publish the Twist message
        self.cmd_vel_pub_.publish(twist_msg)

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
