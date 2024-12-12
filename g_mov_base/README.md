# g_mov_base

Package oriented to integrate the sensors and actuators of the project into the ROS 2 workflow and enable communication with MQTT for Thingspeak, Thingsboard and Mosquitto.

## Connections and physical set up:

The G-Mov model aims to work with ROS 2 robots that work over a Raspberry Pi 3, 4 or 5 that has the camera port available and allow additional GPIO connections. The elements are:

- **ServoMotor MG996r or MG995:** Actuator that will reorientate the camera for better tracking of the person.

- **MPU6050 Module:** Accelerometer/Gyroscope integration for getting feedback on the robot status.

- **Raspberry Pi Cam:** In this case, the Module V2, which is easy to integrate with the Raspberry Pi and aims for the CV application related with the fall detection tracking system.

The connection for this is presented in the image below (Raspberry Pi 4 case):

![g_mov_connection](/g_mov_base/resource/g_mov_connections.png)

In terms of the connection the MPU6050 is connected to the I2C ports of the RPi4, the servo is attached to the GPIO 16 and the cmaera is connected to the proper camera port. All the power connections (5V and GND) are made following the pinout of the RPi (the options presented were added considering ease during installation.) In the case of the Turtlebot3, as it gets power from the RPi 5V port, you can change the MPU6050 to a 3.3 V, move the servo port to the first 5V port and search for another GND to connect the MPU650.

You can 3D print the [g_mov_base_support](/g_mov_description/mesh_files/base_gmov-Body.stl) piece and install all the components there with the appropiate M3 screw for the Turtlebot3 burger model, where you can add the camera in the front or in the back of the robot according to your needs (just keep in mind to change the cmd_vel commands orientation and direction according to the position on the robot, by default, the camara is added at the back of the robot).

## Nodes:

The nodes you can find for this package are presented below:

- **[accel_publisher.py](/g_mov_base/g_mov_base/accel_publisher.py)** : Node that read the I2C connections of the MPU6050, convert it to the provert acceleration and orientation, and publish it to the ROS 2 ecosystem in form of a *AccelStamped* msg.
- **[mosquitto_accel_sub.py](/g_mov_base/g_mov_base/mosquitto_accel_sub.py)** : ROS 2 subscriber that interprates the *AccelStamped* info provided and publish it to the specified Mosquitto broker.
- **[mosquito_fall_sub.py](/g_mov_base/g_mov_base/mosquitto_fall_sub.py)** : ROS 2 subscriber that reads the boolean flag for fall detection and publsih it to the specified Mosquitto broker.
- **[mosquito_servo_sub.py](/g_mov_base/g_mov_base/mosquitto_servo_sub.py)** : ROS 2 subscriber that reads the servo angle and publish it to the specified Mosquitto broker.
- **[servo_subscriber.py](/g_mov_base/g_mov_base/servo_subscriber.py)** : ROS 2 subscriber that folllows the info of a custom *ServoPoseStamped*, interpret it and command the position by using the GPIO ports to the servo.
- **[thingsboatd_accel_sub.py](/g_mov_base/g_mov_base/thingsboard_accel_sub.py)** : ROS 2 subscriber that read the *AccelStamped* info provided and publish it to the specified Thingsboard broker in order to visualize it in a dashboard.
- **[thingsboatd_fall_sub.py](/g_mov_base/g_mov_base/thingsboard_fall_sub.py)** : ROS 2 subscriber that read a boolean flag for fall detection and publish it to the specified Thingsboard broker in order to visualize it in a dashboard.
- **[thingsboatd_servo_sub.py](/g_mov_base/g_mov_base/thingsboard_servo_sub.py)** : ROS 2 subscriber that read the custom *ServoPoseStamped* info provided and publish it to the specified Thingsboard broker in order to visualize it in a dashboard.
- **[thingsboatd_servo_pub.py](/g_mov_base/g_mov_base/thingsboard_servo_sub.py)** : ROS 2 publisher that considers a specified pose in the Thingsboard dashboard and publish the corresponding *ServoPoseStamped* msg.
- **[thingspeak_accel_sub.py](/g_mov_base/g_mov_base/thingspeak_accel_sub.py) : ROS 2 subscriber that aimed to test MQTT connection with Thingspeak from Matlab and explore the usage of this solution during early stages of the project.

## Launch files:

To ease the process of running the sensor and communication nodes, the next launch files were created: 

- **[thingsboard_nodes.launch.py](/g_mov_base/launch/thingsboard_nodes.launch.py)** : Launch file that starts all the Thingsboard nodes for MQTT communication with the dashboard.

- **[sensors_bringup.launch.py](/g_mov_base/launch/sensors_bringup.launch.py)** : Launch files that starts the MPU6050, camera and servo to publish/subscribe in the ROS 2 ecosystem.

- **[mosquitto_nodes.launch.py](/g_mov_base/launch/mosquitto_nodes.launch.py)** : Launch file that initialize the Mosquitto ndoes for MQTT connections with the provided Mosquitto broker.

- **[g_mov_bringup.launch.py](/g_mov_base/launch/g_mov_bringup.launch.py):** Launch files that includes the sensors bringup and thingsboard launch for running the version 1.0 of the *G-Mov Project*.

## Execution:

Currently, the focus on the project is the Thingsboard's Dashboard intregration and the tracking/fall detection module running on the robot.

Before running the robot, make sure the PC hosting the Thingsboard Dashboard and the Thingsboard broker is running. After that you can proceed:

1. Make sure you have cloned the repository:

~~~bash
cd ~/ros2_ws/src
git clone https://github.com/DanielFLopez1620/G-Mov_Project.git
~~~

2. Ensure that all dependenices are available:

~~~bash
pip3 install mpu6050-raspberrypi 
pip3 install rpi-gpio
pip3 install paho-mqtt
pip3 install ssl
pip3 install DateTime
pip install opencv-python
pip install mediapipe
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -y
~~~

3. Ensure that you submitted the proper user/password, hosts and broker credentials before compiling and launching your nodes. This applies for the Mosquitto nodes with the corresponding Mosquitto Broker configuration and the Thingsboard Broker / Database configurations. 

NOTE: If you haven't started your Mosquitto or Thingsboard Services, check [g_mov_local_mqtts_notes](/g_mov_local_mqtts_notes/README.md) for more information.

4. Build the packages

~~~bash
cd ~/ros2_ws
colcon build --packages-select g_mov_msgs g_mov_opencv_py g_mov_base
~~~

5. Run the launch:

~~~bash
source ~/ros2_ws/install/setup.bash
ros2 launch g_mov_base g_mov_bringup.launch.py
~~~
