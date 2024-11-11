# G_MOV_OPENCV_PY

This package is oriented to integrate pose detection and face recognition in order to create a fall detection program.

The implementations provided here are based on the codes of [fall_detection_DL by barkhaarorra and MahiPajuja @ Github](https://github.com/barkhaaroraa/fall_detection_DL) that where adapted to work on a ROS 2 package and modified in order to run better in a Raspberry Pi 4.

## Installation

### Raspberry PI (Ubuntu 22.04):

Supposing you have python 3.10 (or higher) installed, have a ROS 2 Humble installation and you have cloned this  repository, follow the next steps:

1. Update and upgrade your system:

~~~bash
sudo apt update && sudo apt upgrade
~~~

2. Install Python dependencies and requiered libraries

~~~bash
sudo apt install python3-pip python3-dev libatlas-base-dev libhdf5-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libv4l-dev libxvidcore-dev libx264-dev libdc1394-22-dev libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev
~~~

3. Install dlib (required for face_recogniton)

~~~bash
pip3 install dlib
~~~

4. Install computer vision packages

~~~bash
pip3 install opencv-python
pip3 install mediapipe
pip3 install face_recogntion
~~~

5. Run the module example to make sure everythin is fine:

~~~bash
cd /path/to/repo/g_mov_opencv_py/g_mov_opencv_py # Replace with the proper path
python3 modules_demo.py
~~~

### PC (Ubuntu 22.04)

1. Update and upgrade your system:

~~~bash
sudo apt update && sudo apt upgrade
~~~

2. Install computer vision packages

~~~bash
pip3 install opencv-python
pip3 install mediapipe
pip3 install face_recogntion
~~~

3. Run the module example to make sure everythin is fine:

~~~bash
cd /path/to/repo/g_mov_opencv_py/g_mov_opencv_py # Replace with the proper path
python3 modules_demo.py
~~~

## Running the nodes:

1. Do not forget to go to your workspace and build the package

~~~bash
cd /path/to/ros2_ws # Replace with the proper path
colcon build --packages-select g_mov_opencv_py # To build this package only
source install/setup.bash
~~~

2. Run the node:

~~~bash
ros2 run g_mov_opencv_py fall_detect_img_raw_gui.py 
~~~


## Troubleshooting:

- Do not forget to give the proper access to the camera

~~~bash
sudo chmod 666 /dev/video<num_of_camera>
~~~


- Sometimes when downloading multiple computer vision apps, matplotlib can install several times with different version. If you receive warnings related with matplotlib, uninstall and then reinstall it:

~~~bash
pip3 uninstall matplotlib
pip3 install matplotlib
~~~


## Recommended resources:

- The system has a delay of a little more thant two seconds when running on a Raspeberry Pi 4, then it should be interesting to make tests with OpenCV by using C++ as suggests [StereoPi](https://stereopi.com/blog/opencv-comparing-speed-c-and-python-code-raspberry-pi-stereo-vision#:~:text=As%20you%20can%20see%2C%20Python,to%20be%20better%20at%20this) that demostrated it has a better time response for video capture.
- This project uses OpenCV Python, do not forget checking the [documentation](https://docs.opencv.org/3.4/d2/d75/namespacecv.html) if the comments aren't clear.
- This project uses MediaPipe, do not forget checking the [documentation](https://ai.google.dev/edge/mediapipe/solutions/guide) if the comments aren't clear