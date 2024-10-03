#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import mpu6050
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import AccelStamped

class AccelerometerPublisher(Node):
    def __init__(self):
        super().__init__('accelerometer_publisher')
        self.pub = self.create_publisher(AccelStamped, 'accel_info', 10)
        self.timer = self.create_timer(1, self.TimerCallback)
        self.mpu6050 = mpu6050.mpu6050(0x68)
        self.accelerometer_data = []
        self.gyroscope_data = []
        self.temperature_data = []

    def __del__(self):
        pass
    
    def ReadSensorData(self):
        self.accelerometer_data = self.mpu6050.get_accel_data()
        self.gyroscope_data = self.mpu6050.get_gyro_data()
        self.temperature_data = self.mpu6050.get_temp()

    def TimerCallback(self):
        msg = AccelStamped()
        self.ReadSensorData()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "accel"
        msg.accel.linear.x = self.accelerometer_data['x']
        msg.accel.linear.y = self.accelerometer_data['y']
        msg.accel.linear.z = self.accelerometer_data['z']
        msg.accel.angular.x = self.gyroscope_data['x']
        msg.accel.angular.y = self.gyroscope_data['y']
        msg.accel.angular.z = self.gyroscope_data['z']
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    accel_publisher = AccelerometerPublisher()
    rclpy.spin(accel_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()