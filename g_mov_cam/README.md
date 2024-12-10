# g_mov_cam

This was a beta repostiroy to test about camera image transport in search for improvements due to heavy lost when using tandard image msgs from ROS 2.

## Nodes:

Here is presented the list of nodes available:

- [custom_image_transport.cpp](/g_mov_cam/src/custom_image_transport.cpp) : Node oriented to explore image transport with the *image_transport* msg by reading the camera with OpenCV.

## Execution:

1. Make sure you have cloned the repository:

~~~bash
cd ~/ros2_ws/src
git clone https://github.com/DanielFLopez1620/G-Mov_Project.git
~~~

2. Connect camera and check permissions, also validate number of device

~~~bash
v4l2-ctl --list-devices

# For example:
# Microsoft® LifeCam HD-3000: Mi (usb-0000:00:14.0-5):
#         /dev/video0
#         /dev/video1
#         /dev/media0
# Where the camera ID should be 0 or 1
~~~

3. Compile and run

~~~bash
cd ~/ros2_ws
colcon build --packages-select g_mov_cam
source ~/ros2_ws/install/setup.bash
ros2 run g_mov_cam custom_image_transport 0 # Or your camera device number
~~~

4. Check image transport

~~~bash
ros2 topic echo /raw_image/compressed
~~~