#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Based on the fall detection module of barkhaaroraaa and MahiPahuja @ Github:
https://github.com/barkhaaroraa/fall_detection_DL
"""

import face_recognition
import cv2


def recognize_face(frame):
    small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
    rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)  

    face_locations = face_recognition.face_locations(rgb_small_frame)
    face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

    if face_locations:
        return "Face Detected"

if __name__ == "__main__":
    print("Module: Face recognition")
    print("Containts function related to face marking in frames")