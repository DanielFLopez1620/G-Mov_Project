#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# -------------------------- PYTHON REQUIRED LIBRARIES ------------------------
import paho.mqtt.client as mqtt
import ssl
import os
import time
import sys
import json
import random

# ------------------------- ROS2 REQUIRED LIBRARIES ---------------------------
import rclpy
from rclpy.node import Node

# --------------------------- ROS2 REQUIRED MESSAGES --------------------------
from geometry_msgs.msg import AccelStamped

# -------------------- ACCEL SUBSCRIBER TO THINGSPEAK PUBLISHER ---------------
class MosquittoAccelSubs(Node):
    """
    Class that subscriber to the accelerometer topic, obtain the data and then
    publish it to the ThingSpeak platform where is located a MQTT broker with a
    specific format that consider timeStamp, proper sensor and variable ID, the
    value and the measurament.
    """
    def __init__(self):
        """
        Constructor that sets up the communication by using MQTT with paho, to
        connect with the Thinkspeak broker as it provides the proper validation
        to the topic with the user and password generated. It also initialize
        the node with the name "thingspeak_accel_sub", create the proper susbcriber
        with a callback for interpreting the values and initialize some of the
        mqttt params.
        """
        # Initialize node
        super().__init__('thingspeak_accel_subs')

        self.THINGSBOARD_HOST = '192.168.1.162'
        self.ACCESS_TOKEN = '2gImYrCCe2j29PRQcij8'
        self.INTERVAL = 2
        self.sensor_data = {'accel_x' : 0, 'accel_y' : 0, 'accel_z' : 0}
        self.next_reading = time.time()

        

        # Mqtt client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set(self.ACCESS_TOKEN)
        self.mqtt_client.on_connect = on_connect
        self.mqtt_client.on_publish = on_publish
        
        # Make it persistent
        self.mqtt_client.connect(self.THINGSBOARD_HOST, port=1883, keepalive=60)
        self.mqtt_client.loop_start()

        # Initialize values for MQTT transmission
        self.id_sensor = 2
        self.timestamp = 0
        self.unit_sensor = "m/s²"

        # Instance suscription with the callback that will send info to ThingSp
        self.subs = self.create_subscription(AccelStamped, 'accel_info', 
            self.mqtt_callback, 10)

    def __del__(self):
        """
        Destructor to clean up MQTT connection
        """
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()

    def mqtt_callback(self, msg):
        """
        Callback that read the stamp in the value and classifies each linear
        component received to create 3 publication (one for component) that are
        sent to ThingSpeak.
        """
        self.sensor_data['accel_x'] = msg.accel.linear.x
        self.sensor_data['accel_y'] = msg.accel.linear.y
        self.sensor_data['accel_z'] = msg.accel.linear.z

        self.mqtt_client.publish('v1/devices/me/telemetry', json.dumps(self.sensor_data), 1)


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
    accel_to_mqtt = MosquittoAccelSubs()

    # Spin node
    rclpy.spin(accel_to_mqtt)

    # Clean and shutdown
    accel_to_mqtt.destroy_node()
    rclpy.shutdown()   


if __name__ == "__main__":
    main()
