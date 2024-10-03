#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO

import rclpy
from rclpy.node import Node
from g_mov_msgs.msg import ServoPoseStamped

class ServoSubscriber(Node):
    def __init__(self):
        super().__init__('servo_subscriber')
        self.subs = self.create_subscription(ServoPoseStamped, 'servo_angle', self.ServoCallback, 10)
        self.servo_pin = 12
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.servo_pwm = GPIO.PWM(self.servo_pin, 100)
        self.servo_pwm.start(0)
        self.servo_angle = 0
        self.duty = 0

    def __del__(self):
        self.servo_pwm.stop()
        GPIO.cleanup()

    def AngleToDuty(self, ang):
        return float(ang) / 10.0 + 5.0

    def ServoCallback(self, msg):
        self.get_logger().info("Servo update received...\nProcessing...")
        self.servo_angle = msg.data
        self.duty = self.AngleToDuty(self.servo_angle)
        self.servo_pwm.ChangeDutyCycle(self.duty)
        self.get_logger().info("Servo pose updated: %d" % msg.data)


def main(args=None):
    rclpy.init(args=args)
    servo_subscriber = ServoSubscriber()
    rclpy.spin(servo_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()