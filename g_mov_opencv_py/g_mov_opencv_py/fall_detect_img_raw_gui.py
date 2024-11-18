#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ---------------------- STANDARD DEPENDENCIES --------------------------------
import cv2               # OpenCV library for computer vision applications
import mediapipe as mp   # Library oriented for pose detection
import time              # Library related with time

# --------------------- ROS 2 RELATED DEPENDENCIES ----------------------------
import rclpy                    # ROS 2 Client Library for Python
from rclpy.node import Node     # ROS 2 Node class implementation
from cv_bridge import CvBridge  # For traslation of images

# --------------------- ROS 2 MESSAGES INTERFACES -----------------------------
from sensor_msgs.msg import Image   # Image standard msg

# ------------------ CLASS FOR FALL DETECTION (GUI)----------------------------
class FallDetectGUI(Node):
    """
    Class oriented to implement a people fall detection system based on
    mediapipe and cv2.

    Methods
    -----
    listener_callback(msg):
        Callback that respond to a new ROS 2 image to start the fall detection
        system
    
    detect_pose(frame):
        Process a frame with mediapipe in order to obtain a person's landmarks

    detect_fall(landmark, height, should_last_avg_height):
        Based on the landmarks, studies possible falls.
    """
    def __init__(self):
        """
        Initialize a node named "fall_detect_gui" while setting up a image
        subcriber and set up the required params for media pipe and opencv
        to process the fall detection.
        """
        # Initialize node by using parent interface
        super().__init__('fall_detect_gui')
        
        # Create a subscriber to the '/image_raw' topic that calls the
        # "listener_callback" when a new msg is received.
        self.subscription = self.create_subscription( Image,'/image_raw',
            self.listener_callback, 10)
        
        # Bridge to convert ROS image messages to OpenCV
        self.bridge = CvBridge()

        # Set mediapipe solution mode for pose that considers:
        # - static_image_mode : Set as false for video stream input
        # - min_detection_confidence: 0.7 as the value to consider valid a detect.
        # - model_complixity: 2 to refer a more comple landmark accuracy
        self.pose_video = mp.solutions.pose.Pose(static_image_mode=False, 
            min_detection_confidence=0.7, model_complexity=2)

        # Initialize variables 
        self.previous_avg_shoulder_height = 0  # Shoulder height to consider
        self.time1 = time.time()               # Current time

        # Indicate that the subscriber has begun with logs
        self.get_logger().info("Fall detection (GUI) node has been started.")

    def listener_callback(self, msg):
        """
        Callback to process new incoming image in order to make the pose and
        fall detection actions.

        Params
        ---
        msg : sensor_msgs.msg.Image
            Standard ROS 2 Image message
        """
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = cv2.flip(frame, 0)

        # Process the frame for pose detection
        modified_frame, landmarks = self.detect_pose(frame)
        
        # If landmarks were detected...
        if landmarks is not None:
            # Process the landmarks obtained to process the fall
            fall_detected, self.previous_avg_shoulder_height = self.detect_fall(
                landmarks, frame.shape[0], self.previous_avg_shoulder_height)
            
            # If a fall is detected log result
            if fall_detected:
                self.get_logger().info("Fall detected!")

        # Display the modified frame
        cv2.imshow("Pose Detection", modified_frame)

        # Press 'Esc' to exit and close node
        if cv2.waitKey(1) & 0xFF == 27:  
            rclpy.shutdown()

    def detect_pose(self, frame):
        """
        Detect the pose of a person (human) by making a copy and a change of
        format color, then adding the points, lines and boxes to the frame
        in order to display the stimated pose.

        Params
        -----
        frame : cv2 frame
            Frame obtained from a image or a single frame of a video
        
        Returns
        -----
        modified frame : cv2 frame
            Frame that contains the landmarks of the pose detection
        
        landmarks : media pipe pose landmarks
            Raw info of the landmarks
        """

        # Copy frame
        modified_frame = frame.copy()

        # Change color format
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Obtain result from pose stimation
        results = self.pose_video.process(frame_rgb)

        # Get shape of the frame
        height, width, _ = frame.shape

        # Initialize void landmark array
        landmarks = []

        # If there are valid landmarks
        if results.pose_landmarks:
            # For each landmark detect, append in to the landmark array (position)
            for landmark in results.pose_landmarks.landmark:
                landmarks.append(
                    (int(landmark.x * width), int(landmark.y * height),
                        (landmark.z * width)))

            # Mediapipe pose connections set for drawing pose
            connections = mp.solutions.pose.POSE_CONNECTIONS

            # For each connection detect obtain the line and add it to the frame
            for connection in connections:
                start_point = connection[0]
                end_point = connection[1]
                cv2.line(modified_frame, (landmarks[start_point][0],
                    landmarks[start_point][1]),(landmarks[end_point][0], 
                    landmarks[end_point][1]), (0, 255, 0), 3)
        else:
            return modified_frame, None
        return modified_frame, landmarks

    def detect_fall(self, landmarks, height, previous_avg_shoulder_height):
        """
        Detect fall based on the position of the shoulders and validation of some
        previously set thresholds.

        Params
        ----
        landmarks : media pipe pose landmarks
            Pose detected by media pipe in previous processings

        height : int
            Stimated height considered by the opencv frame shape

        previous_av_should_height : int
            Integer with the previous height of the shoulders

        Return 
        ----
        Flag : Boolean
        True if a fall is detected, otherwise it is fall

        previous_av_shoulder_height : int
            Updated value of the height of the shoulders
        """

        # Assign the corresponding landmarks of the shoulders
        left_shoulder_y = landmarks[11][1]
        right_shoulder_y = landmarks[12][1]
        
        # Calculate the average y-coordinate of the shoulder
        avg_shoulder_y = (left_shoulder_y + right_shoulder_y) / 2

        # Zero height shoulder case
        if(previous_avg_shoulder_height== 0):
            previous_avg_shoulder_height = avg_shoulder_y
            return False, previous_avg_shoulder_height
        
        # Fall thresholde set up
        fall_threshold = previous_avg_shoulder_height * 1.5
        print(previous_avg_shoulder_height, avg_shoulder_y,end="\n")
        
        # Check if the average shoulder y-coordinate falls less that previous data
        if avg_shoulder_y > fall_threshold:
            previous_avg_shoulder_height = avg_shoulder_y
            return True, previous_avg_shoulder_height
        else:
            previous_avg_shoulder_height = avg_shoulder_y
            return False, previous_avg_shoulder_height

# ------------------------- MAIN IMPLEMENTATION -------------------------------
def main(args=None):
    """
    Node that detect falls in ROS 2 images that come along with a subscription
    """
    # Initialize ROS 2 Client Library for Python
    rclpy.init(args=args)

    # Instance fall detections classs
    fall_detect_node = FallDetectGUI()
    
    # Manage exceptions (in this case, Keyboard interrupt)
    try:
        # Spin node
        rclpy.spin(fall_detect_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Close stream and finish node
        fall_detect_node.pose_video.close()
        fall_detect_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
