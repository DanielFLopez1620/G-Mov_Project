#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# -------------------- PYTHON RELATED RASPBERRY PI LIBRARIES ------------------
import mpu6050  # Accelerometer library

# ------------------------ ROS 2 DEPENDENCIES ---------------------------------
import rclpy                  # ROS 2 Client Library for Python
from rclpy.node import Node   # Import base class for nodes

# ----------------------- ROS2 MESSAGES REQUIRED ------------------------------
from geometry_msgs.msg import AccelStamped  # Acceleration msg (linear/angular)

# ----------------------- ACCELEROMETER PUBLISHER -----------------------------
class AccelerometerPublisher(Node):
    """
    Publisher oriented to read the lectures of a Accelerometer of type mpu6050
    and make it available to ROS2 communication.
    """

    def __init__(self):
        """
        Constructor that initialize the node with the name 
        "acceleromter_publisher", create the publisher (accelerometer info
        stamped) linked with a timer for a constant publication interval. It
        also initalize the port of the accelerometer and the initial values.
        """
        # Node initialization
        super().__init__('accelerometer_publisher')

        # Timer and pub instance
        self.pub = self.create_publisher(AccelStamped, 'accel_info', 10)
        self.timer = self.create_timer(1, self.TimerCallback)

        # Set the accelerometer port and initialize values
        self.mpu6050 = mpu6050.mpu6050(0x68)
        self.accelerometer_data = {}
        self.gyroscope_data = {}
        self.temperature_data = {}

    def __del__(self):
        """
        Destructor, for now void.
        """
        pass
    
    def ReadSensorData(self) -> None:
        """
        Get and update the sensor values for linear acceleration, gyroscope
        info and temperature of the module.
        """
        self.accelerometer_data = self.mpu6050.get_accel_data()
        self.gyroscope_data = self.mpu6050.get_gyro_data()
        self.temperature_data = self.mpu6050.get_temp()

    def TimerCallback(self) -> None:
        """
        When the timer rise the flag, it call the reading of the components
        and then publish a topic by considering a stamped acceleration (which
        is a ROS2 geometry_msgs) that cosnider linear and angular (gyroscope)
        components, with a stamped time.
        """
        # Instance message and add time stamp
        msg = AccelStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "accel"

        # Get sensor values
        self.ReadSensorData()

        # Update components
        msg.accel.linear.x = self.accelerometer_data['x']
        msg.accel.linear.y = self.accelerometer_data['y']
        msg.accel.linear.z = self.accelerometer_data['z']
        msg.accel.angular.x = self.gyroscope_data['x']
        msg.accel.angular.y = self.gyroscope_data['y']
        msg.accel.angular.z = self.gyroscope_data['z']

        # Publish info
        self.pub.publish(msg)

# ------------------------- MAIN IMPLEMENTATION -------------------------------

def main(args=None):
    """
    Program oriented to publish and make available as a ROS2 topic, the info of
    an accelerometer connected to a Raspberry Pi.
    """

    # Initialize node
    rclpy.init(args=args)

    # Instance publisher
    accel_publisher = AccelerometerPublisher()

    # Spin node
    rclpy.spin(accel_publisher)

    # Close and shutdown
    accel_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()