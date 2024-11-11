#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Based on the fall detection module of barkhaaroraaa and MahiPahuja @ Github:
https://github.com/barkhaaroraa/fall_detection_DL
"""

# ----------------------- STANDERS DEPENDENCIES -------------------------------
import face_recognition  # Oriented to face landmarks and detections
import cv2               # OpenCV library for computer vision applications

# ----------------------- MODULE FUNCTIONS ------------------------------------
def recognize_face(frame):
    """
    Obtain all the faces available, then sends a confirmation message
    """
    # Resize image
    small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

    # Change color format
    rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)  

    # Obtain faces location
    face_locations = face_recognition.face_locations(rgb_small_frame)

    # Encode faces
    face_encodings = face_recognition.face_encodings(rgb_small_frame, 
        face_locations)

    # If a fece is detected, return message
    if face_locations:
        return "Face Detected"

#  ------------------------ MAIN EXECTUION ------------------------------------
if __name__ == "__main__":
    print("Module: Face recognition")
    print("Containts function related to face marking in frames")