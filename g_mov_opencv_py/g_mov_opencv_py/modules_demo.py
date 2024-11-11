#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Simple python node that test the modules of face recognition and fall detection

Based on the fall detection module of barkhaaroraaa and MahiPahuja @ Github:
https://github.com/barkhaaroraa/fall_detection_DL
"""

# -------------------- CUSTOM MODULE DEPENDENCIES -----------------------------
import module_face_recognition  # Custom face recognition functions
import module_fall_detection    # Custom fall detection functions (with marks)

# ------------------------- STANDARD DEPENDENCIES -----------------------------
import cv2              # OpenCV library for Computer Vision
import numpy as np      # Library for math with matrices
import mediapipe as mp  # Computer vision library related with pose detection
from time import time   # Time related functions

# -------------------------- MAIN PROGRAM -------------------------------------
def main():
    """
    Simple program that detects a person pose and face, and by considering the
    shoulder position over time, it detects possible falls.
    """
    # Variable initialization
    previous_avg_shoulder_height = 0  # Previous should height
    time1 = 0                         # Previous time
    fall_detected = False             # Flag for fall detection

    # Mediapipe solution configuration for pose mode that considers:
    # - static_image_mode : Set as false for video stream input
    # - min_detection_confidence: 0.7 as the value to consider valid a detect.
    # - model_complixity: 2 to refer a more comple landmark accuracy
    pose_video = mp.solutions.pose.Pose(static_image_mode=False, 
        min_detection_confidence=0.7, model_complexity=2)

    # Obtain video from /dev/video0 (must have proper permissions)
    video = cv2.VideoCapture(0)

    # Main loop that will develop the action while the camera is broadcasting
    while video.isOpened():

        # Obtain current frame
        ret, frame = video.read()

        # If no frames where received, skip loop
        if not ret:
            break
        
        # Detect pose call (check module_fall_detection for more info)
        modified_frame, landmarks = module_fall_detection.detectPose(
            frame, pose_video, display=True)
        
        # Recognize face call (check module_face_recognition for more info)
        face_names = module_face_recognition.recognize_face(frame)
        
        # If a face is detected, display message received
        if face_names is not None:
            print("Detected faces:", face_names)
        
        # Get current time
        time2 = time()

        # Validate if there was certain time pass
        if (time2 - time1) > 2:
            # If a pose (body) was detected
            if landmarks is not None:
                # Obtain shape values (size)
                height, _, _ = frame.shape

                # Call fall detection (more info on the proper module)
                fall_detected, previous_avg_shoulder_height = module_fall_detection.detectFall(landmarks,
                    height, previous_avg_shoulder_height)

                # If a fall is detected, display to know
                if fall_detected:                 
                    print("Fall detected!")  

            # Update times        
            time1 = time2
        
        # If key for quit is recieved, then exit loop
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break

    # Kill and exit cv2 windows.
    video.release()
    cv2.destroyAllWindows()

# Main program execution
if __name__ == "__main__":
    main()