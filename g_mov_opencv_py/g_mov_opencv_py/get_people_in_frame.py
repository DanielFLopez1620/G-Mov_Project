import cv2
import mediapipe as mp
import time

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image   # Image standard msg
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class FallDetectNoGUI(Node):
    def __init__(self):
        super().__init__('fall_detect_no_gui')
        self.img_subs_ = self.create_subscription(Image, '/image_raw', self.listener_callback, 10)
        
        self.fall_pub_ = self.create_publisher(Bool, '/fall_detect', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)  # Publisher for robot movement
        
        self.bridge = CvBridge()
        self.pose_video = mp.solutions.pose.Pose(static_image_mode=False, min_detection_confidence=0.7, model_complexity=2)
        
        self.previous_avg_shoulder_height = 0  # Shoulder height to consider
        self.time1 = time.time()               # Current time
        self.target_person_height = 0.4  # Target height in image (as a fraction of image height)
        self.margin = 0.05  # Allowable margin for person height to be in frame
        
        self.get_logger().info("Fall detection (NO GUI) node has been started.")

    def listener_callback(self, msg):
        flag_fall = Bool()
        flag_fall.data = False
        
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = cv2.flip(frame, 0)
        
        modified_frame, landmarks = self.detect_pose(frame)
        
        if landmarks is not None:
            # Detect fall
            fall_detected_, self.previous_avg_shoulder_height = self.detect_fall(
                landmarks, frame.shape[0], self.previous_avg_shoulder_height)
            
            if fall_detected_:
                self.get_logger().info("Fall detected!")
                flag_fall.data = True
                self.fall_pub_.publish(flag_fall)
            else:
                self.fall_pub_.publish(flag_fall)
            
            # Adjust robot movement to maintain person's height in frame
            self.get_logger().info("Adjusting pose...")
            self.adjust_distance(landmarks, frame.shape[0])

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
        print(previous_avg_shoulder_height, avg_shoulder_y, end="\n")
        
        if avg_shoulder_y > fall_threshold:
            previous_avg_shoulder_height = avg_shoulder_y
            return True, previous_avg_shoulder_height
        else:
            previous_avg_shoulder_height = avg_shoulder_y
            return False, previous_avg_shoulder_height

    def adjust_distance(self, landmarks, frame_height):
        # Use head and ankle landmarks to estimate the person’s height in the frame
        head_y = landmarks[0][1]  # Assume landmark 0 is the head
        ankle_y = landmarks[29][1]  # Assume landmark 29 is the left ankle
        
        person_height = abs(ankle_y - head_y) / frame_height  # Person height as fraction of frame height
        
        twist = Twist()
        
        if person_height < (self.target_person_height - self.margin):
            # Person is too far away, move forward
            twist.linear.x = 0.2
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info("Moving forward to keep person in frame.")
        
        elif person_height > (self.target_person_height + self.margin):
            # Person is too close, move backward
            twist.linear.x = -0.2
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info("Moving backward to keep person in frame.")
        
        else:
            # Person is at the desired distance, stop moving
            twist.linear.x = 0.0
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info("Not moving...")

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
