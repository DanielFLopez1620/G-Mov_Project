#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Based on the fall detection module of barkhaaroraaa and MahiPahuja @ Github:
https://github.com/barkhaaroraa/fall_detection_DL
"""

import module_face_recognition
import module_fall_detection

import cv2
import numpy as np
import mediapipe as mp
from time import time

def main():
    previous_avg_shoulder_height = 0

    pose_video = mp.solutions.pose.Pose(static_image_mode=False, min_detection_confidence=0.7, model_complexity=2)
    video = cv2.VideoCapture(0)
    time1 = 0
    fall_detected = False
        
    while video.isOpened():
        ret, frame = video.read()
        if not ret:
            break
        
        modified_frame, landmarks = module_fall_detection.detectPose(frame, pose_video, display=True)
        face_names = module_face_recognition.recognize_face(frame)
        if face_names is not None:
            print("Detected faces:", face_names)
            
        time2 = time()
        if (time2 - time1) > 2:
            # print("time")
            if landmarks is not None:
                # print("landmarks")
                height, _, _ = frame.shape
                fall_detected, previous_avg_shoulder_height = module_fall_detection.detectFall(landmarks, height, previous_avg_shoulder_height)
                if fall_detected:                 
                    print("Fall detected!")          
            time1 = time2
        k = cv2.waitKey(1) & 0xFF
        if k == 27:
            break

    video.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()