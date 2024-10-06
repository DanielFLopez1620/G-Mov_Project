#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# -------------------------- PYTHON REQUIRED LIBRARIES -----------------------
import paho.mqtt.publish as publish

# -------------------------- ROS2 REQUIRED LIBRARIES -------------------------
import rclpy
from rclpy.node import Node

# -------------------------- ROS2 REQUIRED MESSAGES ---------------------------
from g_mov_msgs.msg import ServoPoseStamped

# ------------------- SERVO SUBSCRIBER TO THINGSPEAK PUBLISHER ----------------
class ThingSpeakServoSubs(Node):
    """
    Class that subscriber to the servo topic, obtain the data and then publish
    it to the ThingSpeak platform where is located a MQTT broker with a 
    specific format that consider timeStamp, proper sensor and variable ID, the
    value and the measurament.
    """
    def __init__(self):
        """
        Constructor that sets up the communication by using MQTT with paho, to
        connect with the Thinkspeak broker as it provides the proper validation
        to the topic with the user and password generated. It also initialize
        the node with the name "thingspeak_servo_sub", create the proper 
        susbcriber with a callback for interpreting the values and initialize
        some of the mqtt params.
        """
        # Initialize node
        super().__init__('thingspeak_servo_subs')

        # Set thingspeak params for MQTT connection
        self.channel_id = "2667301"
        self.mqtt_client_ID = "ER0GJDQkIQsBNR0hGjcqIyw"
        self.mqtt_username = "ER0GJDQkIQsBNR0hGjcqIyw"
        self.mqtt_password = "Fu4VIaTN2ZtaoqTuMUnGYv5D"
        self.mqtt_topic = "channels/" + self.channel_id + "/publish"
        self.mqtt_host = "mqtt3.thingspeak.com"

        # Initialize values for MQTT transmission
        self.id_sensor = 1
        self.variable_sensor = 1
        self.timestamp = 0
        self.unit_sensor = "grados"
        self.payload = ""

        # Instance suscription with the callbak that will send info to ThingSp
        self.subs = self.create_subscription(ServoPoseStamped, 'servo_angle',
            self.mqtt_callback, 10)

    def __del__(self):
        """
        Destructor that is void for now
        """
        pass
        
    def mqtt_callback(self, msg):
        """
        Callback that read the stamp and the float value of the servo, then
        it publish to MQTT.
        """
        # Get stamp
        self.timestamp = msg.header.stamp

        # Update payload with the proper fields order
        self.payload = "field1=" + str(self.id_sensor) 
        self.payload += "&field2=" + str(self.timestamp) 
        self.payload += "&field3=" + str(self.variable_sensor) 
        self.payload += "&field4=" + str(msg.data) 
        self.payload += "&field5=" + str(self.unit_sensor)

        # Single publish to ThingSpeak
        publish.single(self.mqtt_topic,
                payload= self.payload,
                hostname= self.mqtt_host,
                transport= "tcp",
                port= 1883,
                client_id= self.mqtt_client_ID,
                auth = {
                        'username':self.mqtt_username,
                        'password': self.mqtt_password
                        }
                )
        
        # Clear values before cycles start agian.
        self.payload = ""

# ------------------------- MAIN IMPLEMENTATION -------------------------------

def main(args=None):
    """
    Program that receives info of the servo topic and sent it to the
    Thinkspeak platform where a the broker is located.
    """
    # Initialize node
    rclpy.init(args=args)

    # Instance subscriber
    servo_pose_to_mqtt = ThingSpeakServoSubs()

    # Spin node
    rclpy.spin(servo_pose_to_mqtt)

    # Clean and shutdown
    servo_pose_to_mqtt.destroy_node()
    rclpy.shutdown()   


if __name__ == "__main__":
    main()