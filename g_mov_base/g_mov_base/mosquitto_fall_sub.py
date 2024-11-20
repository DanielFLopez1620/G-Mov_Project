#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# -------------------------- PYTHON REQUIRED LIBRARIES ------------------------
import paho.mqtt.client as mqtt   # Library for MQTT connections
import ssl                        # Related with secure TLS
import datetime                   # For time stamps and dates

# ------------------------- ROS 2 REQUIRED LIBRARIES --------------------------
import rclpy                     # ROS 2 Client Library for Python 
from rclpy.node import Node      # Base class for nodes

# --------------------------- ROS 2 REQUIRED MESSAGES -------------------------
from std_msgs.msg import Bool    # Boolean standard message for ROS 2

# -------------------- FALL SUBSCRIBER TO MOSQUITTO PUBLISHER ---------------
class MosquittoFallSubs(Node):
    """
    Class that subscriber to the fall flag topic, obtain the data and then
    publish it to the Mosquitto local broker by using a specific format that 
    considers the timeStamp, proper sensor and variable ID, the value and the 
    measurament.
    """
    def __init__(self):
        """
        Constructor that sets up the communication by using MQTT with paho, to
        connect with the Thinkspeak broker as it provides the proper validation
        to the topic with the user and password generated. It also initialize
        the node with the name "mosquitto_fall_sub", create the proper 
        susbcriber with a callback for interpreting the values and initialize 
        some of the mqttt params.
        """
        # Initialize node
        super().__init__('mosquitto_fall_sub')

        # Set Mosquitto params for MQTT with TLS connection over Mosquitto
        self.MQTT_BROKER = "mqtt.local"
        self.MQTT_PORT = 8883
        self.MQTT_TOPIC = "sensors/fall"
        self.MQTT_CLIENT_ID = "mqtt_pub_fall_client"
        self.MQTT_CA_CERT = "/etc/mosquitto/ca_certificates/ca_host.crt"
        self.MQTT_USER = "Dan1620"
        self.MQTT_PASSWORD = "h1d4n16"

        # Instance mosquitto client with proper auth and certificates
        self.mqtt_client = mqtt.Client(client_id=self.MQTT_CLIENT_ID)
        self.mqtt_client.tls_set(ca_certs=self.MQTT_CA_CERT, 
            tls_version=ssl.PROTOCOL_TLSv1_2)
        self.mqtt_client.username_pw_set(self.MQTT_USER, self.MQTT_PASSWORD)
        
        # Set connection and publish callbacks
        self.mqtt_client.on_connect = on_connect
        self.mqtt_client.on_publish = on_publish
        
        # Make the connection and set it up persistent
        self.mqtt_client.connect(self.MQTT_BROKER, port=self.MQTT_PORT, 
            keepalive=60)
        self.mqtt_client.loop_start()

        # Initialize values for MQTT transmission
        self.ID_SENSOR = 4           # Fall detection is sensor_id = 4
        self.timestamp = 0           # Initalize null timestamp
        self.UNIT_SENSOR = "boolean" # Specification of boolean type
        self.VALUE_ID = 0            # Fall detection has no other components

        # Instance suscription with the callback that will send info to ThingSp
        self.subs = self.create_subscription(Bool, '/fall_detect',
            self.mqtt_callback, 10)

    def __del__(self):
        """
        Destructor to clean up MQTT connection to the Mosquitto Broker
        """
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()

    def mqtt_callback(self, msg) -> None:
        """
        Callback that read tha considers current time and boolean flag of fall,
        then send the information to the Mosquitto broker.

        Params
        ---
        msg : std_msgs.msg.Bool
            Message received from accelerometer node 
        """
        # Get stamp
        self.timestamp = datetime.datetime.now()

        # Obtain components
        self.value = msg.data

        # Update payload with the proper fields order
        payload = "field1=" + str(self.ID_SENSOR) 
        payload += "&field2=" + str(self.timestamp) 
        payload += "&field3=" + str(self.VALUE_ID) 
        payload += "&field4=" + str(self.value) 
        payload += "&field5=" + str(self.UNIT_SENSOR)

        # Publish to MQTT
        flag = self.mqtt_client.publish(self.MQTT_TOPIC, payload)
        flag.wait_for_publish()

# Callback function on connect
def on_connect(client, userdata, flags, rc) -> None:
    """
    On Mqtt connecction, notify and create a log to specify if it was
    succesful or returned an error code.

    Some common errors are timed out (due to connection limits or being in a
    different network) or invalid data (password/certificates)

    Params
    ---
    client : paho.mqtt.client.Client
        Instance client for the callback
    
    userdata : paho.mqtt.client.Client.user_data_set()
        User defined data in client
    
    flags : paho.mqtt.client.ConnectFlags
        Flags according the connection

    rc : paho.mqtt.reasoncodes.ReasonCode
        Codes for identification of errors
    """
    if rc == 0:
        print("Connected to broker successfully")
    else:
        print(f"Failed to connect, return code {rc}")

# Callback function on publish
def on_publish(client, userdata, mid) -> None:
    """
    If a message is published, log the current number of messages by using mid.

    Params
    ---
    client : paho.mqtt.client.Client
        Instance client for the callback
    
    userdata : paho.mqtt.client.Client.user_data_set()
        User defined data in client

    mid : int
        Mid variable returned form the publish
    """
    print(f"Message published with mid: {mid}")

# --------------------------- MAIN IMPLEMENTATION -----------------------------
def main(args=None):
    """
    Program that receives info of the fall flag topic and sent it to the
    Mosquitto local broker.
    """
    # Initialize node
    rclpy.init(args=args)

    # Instance subscriber
    fall_to_mqtt = MosquittoFallSubs()

    # Manage keyboard exception
    try:
        # Spin node
        rclpy.spin(fall_to_mqtt)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean and shutdown
        fall_to_mqtt.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
