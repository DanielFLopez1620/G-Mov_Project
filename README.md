# G-Mov Project

G-Mov comes from Guardian of Movement. It is a project that aims to integrates robotics in the care of elderly people by combining ROS 2 and IoT for a dashbaord visualization over a local network. By adding a modular kit composed of a pi camera, a servo and a accelerometer to track people fall detection by implementing Computer Vision (CV) with the movement of a robot.

![g_mov_module](/resources/g_mov_module.png)

For this first version, we have used the [Turtlebot3 Burger](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/). You can find a demo video down below:

![g_mov_short_video](/resources/g_mov_project_short_video.gif)

This video was recorded at double speed. If you want to check the original version, you can go to the [G-Mov Phase 3 | Youtube Video](https://youtu.be/IMeLtWwy5p4)

## Technical Information:

This project combines ROS 2 nodes with communication over MQTT by using [Mosquitto](https://mosquitto.org/) and [Thingsboard](https://thingsboard.io/) to generate an interaction between the ROS 2 ecosystem, scripts for CV and communication, a website and a dashboard.

As a note, for now the CV and image processing is made inside the raspberry due to lossy transport of images.

The structure of the project is shown below:

### Packages:

- **[g_mov_base:](/g_mov_base/README.md)** Package with the source code for running the sensors and actuators, and communicating the info over ROS 2 workflow and also with MQTT for Mosquitto, ThingsSpeak and Thingsboard. It also contains the hardware elements and assembly considerations.
- **[g_mov_description:](/g_mov_description/README.md)** Directory that contains the CAD files made in [FreeCad](https://www.freecad.org/) to allow modifications and the .stl files for printing them.
- **[g_mov_cam:](/g_mov_cam/README.md)** Beta package that has experiments for image compression and image transport as it fails and doesn't allow for using the image in another remote ROS 2 system.
- **[g_mov_local_mqtts_notes:](/g_mov_local_mqtts_notes/README.md)** Contains the proper explanation on how to set up Mosquitto broker (base and TLS) and Thingsboard (base installation) for the communication over a local network that must be accesible to the robot and at least another PC.
- **[g_mov_msgs:](/g_mov_msgs/README.md)** Pacakge that contains custom messages, in this case, oriented for the servo pose with a stamped time.
- **[g_mov_opencv_py:](/g_mov_opencv_py/README.md)** Package that incoporates [OpenCV](https://opencv.org/) and [MediaPipe](https://ai.google.dev/edge/mediapipe/solutions/guide) for image processing and CV to detect people fall detection.
- **[g_mov_webpage](/g_mov_webpage/README.md):** Directory that contains the information of the Website that was made with [Flask](https://flask.palletsprojects.com/en/stable/), HTML, CSS and [Bootstrap](https://getbootstrap.com/).


## Limitations:

For now, the project works only with Turtlebot3 and runs with Ubuntu 22.04 for ROS 2 Humble Distro. This was a project made for the IoT Course for the Pontificia Universidad Javeriana, so another limitation was the time to work on the project.

Some additional constraints and limitations to consider are:

- Although it is a IoT project, it just works over a local network as the authors didn't integrate a network host or domain for the webpage due to time limitations.
- The tracking of the person, just works on open spaces where there are not walls where the person could hide and the robot is only able to move in spaces that doesn't have stairs or important height changes. It is possible to detect some parts of the body if the person is behind a partial obstacle but this may affect the tracking.
- Most of the processing is running on the raspberry, so according to the model, it may go slower.

## Future work:

Next semester, according to the develop of the career of the members, have the mission to accomplish one or more of the next items:

* Make a modular design of the kit to allow integration with other robots, for example, [DiffDan](https://github.com/DanielFLopez1620/diff_robot_dan_ros)
* Integrate Docker for easy deployment / installation of the Dashboard and connections.
* Explore integration with SLAM / Nav 2 packages for a more powerful people tracking algorithm in closed spaces.
* Study alternatives for image transport

## About the Authors:

This project was originally made by:

- Daniel Felipe López Escobar : [DanielFLopez1620](https://github.com/DanielFLopez1620)

- Guillermo Aguilera [Best-Gagil](https://github.com/Best-Gagil)

- Cristhian Jose Narvaez. [Criss19](https://github.com/Crisss19)

This project was made by members of the [IEEE RAS Pontificia Unversidad Javeriana Student Chapter](https://linktr.ee/rasjaverianaieee). We would like to thank for the unconditional support and the loan of Turtlebot3 to the directives and members of the Student Chapter.
