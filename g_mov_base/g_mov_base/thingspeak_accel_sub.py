#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# -------------------------- PYTHON REQUIRED LIBRARIES ------------------------
import paho.mqtt.client as mqtt

# ------------------------- ROS 2 REQUIRED LIBRARIES --------------------------
import rclpy
from rclpy.node import Node

# --------------------------- ROS 2 REQUIRED MESSAGES -------------------------
from geometry_msgs.msg import AccelStamped

# -------------------- ACCEL SUBSCRIBER TO THINGSPEAK PUBLISHER ---------------
class ThingSpeakAccelSubs(Node):
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

        # Set thingspeak params for MQTT connection
        self.channel_id = "2667301"
        self.mqtt_client_ID = "ER0GJDQkIQsBNR0hGjcqIyw"
        self.mqtt_username = "ER0GJDQkIQsBNR0hGjcqIyw"
        self.mqtt_password = "Fu4VIaTN2ZtaoqTuMUnGYv5D"
        self.mqtt_topic = "channels/" + self.channel_id + "/publish"
        self.mqtt_host = "mqtt3.thingspeak.com"

        # Persistent client
        self.mqtt_client = mqtt.Client(client_id=self.mqtt_client_ID)
        self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)
        self.mqtt_client.connect(self.mqtt_host, port=1883, keepalive=60)
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
        # Get stamp
        self.timestamp = msg.header.stamp

        # Obtain components
        accel_val= (msg.accel.linear.x, msg.accel.linear.y, msg.accel.linear.z)
        counter = 1

        # Iterate to publish each component
        for val in accel_val:
            # Update payload with the proper fields order
            payload = "field1=" + str(self.id_sensor) 
            payload += "&field2=" + str(self.timestamp) 
            payload += "&field3=" + str(counter) 
            payload += "&field4=" + str(val) 
            payload += "&field5=" + str(self.unit_sensor)

            # Publish to MQTT
            self.mqtt_client.publish(self.mqtt_topic, payload)

            # Update values before cycle start again
            counter += 1

# --------------------------- MAIN IMPLEMENTATION -----------------------------
def main(args=None):
    """
    Program that receives info of the accelerometer topic and sent it to the
    Thinkspeak platform where a the broker is located.
    """
    # Initialize node
    rclpy.init(args=args)

    # Instance subscriber
    accel_to_mqtt = ThingSpeakAccelSubs()

    # Spin node
    rclpy.spin(accel_to_mqtt)

    # Manage keyboard exception
    try:
        # Spin node
        rclpy.spin(accel_to_mqtt)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean and shutdown
        accel_to_mqtt.destroy_node()
        rclpy.shutdown()  
     


if __name__ == "__main__":
    main()
