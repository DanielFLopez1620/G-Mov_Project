#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# -------------------------- PYTHON REQUIRED LIBRARIES ------------------------
import paho.mqtt.client as mqtt
import ssl
import datetime

# ------------------------- ROS2 REQUIRED LIBRARIES ---------------------------
import rclpy
from rclpy.node import Node

# --------------------------- ROS2 REQUIRED MESSAGES --------------------------
from std_msgs.msg import Bool

# -------------------- FALL SUBSCRIBER TO MOSQUITTO PUBLISHER ---------------
class MosquittoFallSubs(Node):
    """
    Class that subscriber to the accelerometer topic, obtain the data and then
    publish it to the Mosquitto platform where is located a MQTT broker with a
    specific format that cimport datatime and consider the timeStamp, proper 
    sensor and variable ID, thevalue and the measurament.
    """
    def __init__(self):
        """
        Constructor that sets up the communication by using MQTT with paho, to
        connect with the Thinkspeak broker as it provides the proper validation
        to the topic with the user and password generated. It also initialize
        the node with the name "mosquitto_fall_sub", create the proper susbcriber
        with a callback for interpreting the values and initialize some of the
        mqttt params.
        """
        # Initialize node
        super().__init__('mosquitto_fall_subs')

        # Set Mosquitto params for MQTT with TLS connection over Mosquitto
        self.MQTT_BROKER = "mqtt.local"
        self.MQTT_PORT = 8883
        self.MQTT_TOPIC = "sensors/fall"
        self.MQTT_CLIENT_ID = "mqtt_pub_fall_client"
        self.MQTT_CA_CERT = "/etc/mosquitto/ca_certificates/ca_host.crt"
        self.MQTT_USER = "Dan1620"
        self.MQTT_PASSWORD = "h1d4n16"

        # Instance client
        self.mqtt_client = mqtt.Client(client_id=self.MQTT_CLIENT_ID)
        self.mqtt_client.tls_set(ca_certs=self.MQTT_CA_CERT, 
            tls_version=ssl.PROTOCOL_TLSv1_2)
        self.mqtt_client.username_pw_set(self.MQTT_USER, self.MQTT_PASSWORD)
        self.mqtt_client.on_connect = on_connect
        self.mqtt_client.on_publish = on_publish
        
        # Make it persistent
        self.mqtt_client.connect(self.MQTT_BROKER, port=self.MQTT_PORT, 
            keepalive=60)
        self.mqtt_client.loop_start()

        # Initialize values for MQTT transmission
        self.id_sensor = 4           # Fall detection is sensor_id = 4
        self.timestamp = 0           # Initalize null timestamp
        self.unit_sensor = "boolean" # Specification of boolean type
        self.value_id = 0            # Fall detection has no other components

        # Instance suscription with the callback that will send info to ThingSp
        self.subs = self.create_subscription(Bool, '/fall_detect',
            self.mqtt_callback, 10)

    def __del__(self):
        """
        Destructor to clean up MQTT connection to the Mosquitto Broker
        """
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()

    def mqtt_callback(self, msg):
        """
        Callback that read the stamp in the value and classifies each linear
        component received to create 3 publication (one for component) that are
        sent to Mosquitto.
        """
        # Get stamp
        self.timestamp = datetime.datetime.now()

        # Obtain components
        self.value = msg.data

        # Update payload with the proper fields order
        payload = "field1=" + str(self.id_sensor) 
        payload += "&field2=" + str(self.timestamp) 
        payload += "&field3=" + str(self.value_id) 
        payload += "&field4=" + str(self.value) 
        payload += "&field5=" + str(self.unit_sensor)

        # Publish to MQTT
        flag = self.mqtt_client.publish(self.MQTT_TOPIC, payload)
        flag.wait_for_publish()

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
    accel_to_mqtt = MosquittoFallSubs()

    # Spin node
    rclpy.spin(accel_to_mqtt)

    # Clean and shutdown
    accel_to_mqtt.destroy_node()
    rclpy.shutdown()   


if __name__ == "__main__":
    main()
