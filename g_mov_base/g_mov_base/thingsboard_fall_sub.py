#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# -------------------------- PYTHON REQUIRED LIBRARIES ------------------------
import paho.mqtt.client as mqtt
import json

# ------------------------- ROS2 REQUIRED LIBRARIES ---------------------------
import rclpy
from rclpy.node import Node

# --------------------------- ROS2 REQUIRED MESSAGES --------------------------
from std_msgs.msg import Bool

# ------------------- FALL SUBSCRIBER TO THINGSBOARD PUBLISHER ---------------
class ThingsboardAccelSubs(Node):
    """
    Class that subscriber to the accelerometer topic, obtain the data and then
    publish it to the ThingSBoard platform where is located a MQTT broker and
    allow the option to graph the content.
    """
    def __init__(self):
        """
        Constructor that sets up the communication by using MQTT with paho, to
        connect with the Thingsboard broker as it provides the proper validation
        to the topic with the user and password generated. It also initialize
        the node with the name "thingsboard_fall_sub", create the proper susbcriber
        with a callback for interpreting the values and initialize some of the
        mqttt params.
        """
        # Initialize node
        super().__init__('thingsboard_servo_subs')

        # Set parameters for connection
        self.THINGSBOARD_HOST = 'mqtt.local'
        self.ACCESS_TOKEN = '2gImYrCCe2j29PRQcij8'
        self.MQTT_TOPIC = 'v1/devices/me/telemetry'

        # Use dict to pass the data
        self.sensor_data = {'servo' : 0}

        # Mqtt client connection
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set(self.ACCESS_TOKEN)
        self.mqtt_client.on_connect = on_connect
        self.mqtt_client.on_publish = on_publish
        
        # Make it persistent
        self.mqtt_client.connect(self.THINGSBOARD_HOST, port=1883, keepalive=60)
        self.mqtt_client.loop_start()

        # Instance suscription with the callback that will send info to ThingSp
        self.subs = self.create_subscription(Bool, '/fall_detect',
            self.mqtt_callback, 10)

    def __del__(self):
        """
        Destructor to clean up MQTT connection to the ThingsBoard broker
        """
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()

    def mqtt_callback(self, msg):
        """
        Callback that read the stamp in the value and classifies each linear
        component received to create a servo angle publication to ThingsBoard
        """
        # Assgin components to the dict
        self.sensor_data['fall'] = msg.data

        # Publish it to the specified topic
        self.mqtt_client.publish(self.MQTT_TOPIC, json.dumps(self.sensor_data), 1)


# Callback function on connect
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to broker successfully")
    else:
        print(f"Failed to connect, return code {rc}")

# Callback function on publish
def on_publish(client, userdata, mid):
    print(f"Message published with mid: {mid}")

# --------------------------- MAIN IMPLEMENTATION -----------------------------
def main(args=None):
    """
    Program that receives info of the accelerometer topic and sent it to the
    Thinkspeak platform where a the broker is located.
    """
    # Initialize node
    rclpy.init(args=args)

    # Instance subscriber
    accel_to_mqtt = ThingsboardAccelSubs()

    # Spin node
    rclpy.spin(accel_to_mqtt)

    # Clean and shutdown
    accel_to_mqtt.destroy_node()
    rclpy.shutdown()   


if __name__ == "__main__":
    main()
