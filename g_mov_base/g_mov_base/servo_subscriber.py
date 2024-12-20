#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ------------------ PYTHON RELATED RASPBERRY PI LIBRARIES --------------------
import RPi.GPIO as GPIO  # General Input/Output Port Management

# ---------------------------- ROS 2 DEPENDENCIES -----------------------------
import rclpy                  # ROS 2 Client Library for Python
from rclpy.node import Node   # Base class for nodes

# -------------------------- ROS 2 REQUIRED MESSAGES --------------------------
from g_mov_msgs.msg import ServoPoseStamped  # Custom servo message

# -------------------------- SERVO ROS2 SUBSCRIBER ----------------------------
class ServoSubscriber(Node):
    """
    Subscriber that receives angular position and set up the servo position by
    using GPIO ports.
    """
    def __init__(self):
        """
        Constructor that initialize the node with the name "servo subscriber",
        create a subscriber linked with a callback to set the servo angle and
        initialize/configure the port and pwm for the servo.
        """
        # Node initialization
        super().__init__('servo_subscriber')

        # Instance subscriber and link callback
        self.subs = self.create_subscription(ServoPoseStamped, 'servo_angle', 
            self.ServoCallback, 10)
        
        # Configure GPIO and PWM
        self.servo_pin = 16
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.servo_pwm = GPIO.PWM(self.servo_pin, 100)

        # Initialize values
        self.servo_pwm.start(0)
        self.servo_angle = 0
        self.duty = 0

    def __del__(self):
        """
        Destructor that cleanup GPIO port usage and stop the PWM
        """
        self.servo_pwm.stop()
        GPIO.cleanup()

    def AngleToDuty(self, ang) -> float:
        """
        Method that convert from angle to duty cycle for the PWM required to
        set the servo position

        Params
        ----
        ang : float
            Angular position objective

        Return
        ---
        Duty cycle of the given objective
        """
        return float(ang) / 10.0 + 5.0

    def ServoCallback(self, msg) -> None:
        """
        Callback that read the data of the servo position, then process the
        correct PWM to achieve the position and make the proper log.

        Params
        ----
        msg : g_mov_msgs.msg.ServoStamped
            Servo angle msg received
        """
        self.get_logger().info("Servo update received...\nProcessing...")
        
        # Get servo angle value
        self.servo_angle = msg.data

        # Set position
        self.duty = self.AngleToDuty(self.servo_angle)
        self.servo_pwm.ChangeDutyCycle(self.duty)
        
        self.get_logger().info("Servo pose updated: %d" % msg.data)

# -------------------------------- MAIN IMPLEMENTATION ------------------------

def main(args=None):
    """
    Program oriented to subscribe to obtain the servo position and produce it
    by considering GPIO port with the proper PWM.
    """
    # Initialize node
    rclpy.init(args=args)

    # Instance publisher
    servo_subscriber = ServoSubscriber()

    # Manage keyboard exception
    try:
        # Spin node
        rclpy.spin(servo_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean and shutdown
        servo_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
