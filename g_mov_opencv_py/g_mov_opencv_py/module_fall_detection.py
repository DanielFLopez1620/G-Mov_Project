#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Based on the fall detection module of barkhaaroraaa and MahiPahuja @ Github:
https://github.com/barkhaaroraa/fall_detection_DL
"""
# -------------------------- STANDARD DEPENDENCIES ----------------------------
import cv2                # OpenCV library for Computer Vision applications
import mediapipe as mp    # Library oriented for pose detections


# -------------------------- MODULE FUNCTIONS ---------------------------------
def detectPose(frame, pose_model, display=True):
    """
    Detect the pose of a person (human) by making a copy and a change of
    format color, then adding the points, lines and boxes to the frame
    in order to display the stimated pose.

    Params
    -----
    frame : cv2 frame
        Frame obtained from a image or a single frame of a video
    
    pose_model : mediapipe solution
        Model configuration for the media pipe pose stimation

    display : boolean
        Confirm cv2 visual window, by default it is true.

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
    results = pose_model.process(frame_rgb)

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

    # If now frame with pose is detected, return nothing
    else:
        return None, None 
    
    # If display flag is active, show results
    if display:
        cv2.imshow('Pose Landmarks', modified_frame)
    
    # Return landmarks 
    return modified_frame, landmarks

def detectFall(landmarks, height, previous_avg_shoulder_height):
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


# -------------------- MAIN EXECUTION -----------------------------------------
if __name__ == "__main__":
    print("Module: Fall Detection utilities")
    print("Containts function related to process frames and indicate falls.")