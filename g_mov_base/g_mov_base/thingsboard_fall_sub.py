#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# -------------------------- PYTHON REQUIRED LIBRARIES ------------------------
import paho.mqtt.client as mqtt   # Library for MQTT connections
import json                       # Json formatting 

# ------------------------- ROS 2 REQUIRED LIBRARIES --------------------------
import rclpy                     # ROS 2 Client Library for Python 
from rclpy.node import Node      # Base class for nodes

# --------------------------- ROS 2 REQUIRED MESSAGES -------------------------
from std_msgs.msg import Bool    # Boolean standard message

# ------------------- FALL SUBSCRIBER TO THINGSBOARD PUBLISHER ---------------
class ThingsboardFallSubs(Node):
    """
    Class that subscriber to the fall flag topic, obtain the data and then
    publish it to the ThingsBoard platform where is located a MQTT broker and
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
        super().__init__('thingsboard_fall_sub')

        # Set parameters for connection
        self.THINGSBOARD_HOST = 'mqtt.local'
        self.ACCESS_TOKEN = '<your_access_token>'
        self.MQTT_TOPIC = 'v1/devices/me/telemetry'

        # Use dict to pass the data, that will be then converted in json
        self.sensor_data = {'servo' : 0}

        # Mqtt client connection for local hosted Thingsboard service
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set(self.ACCESS_TOKEN)

        # Set connection and publish callbacks
        self.mqtt_client.on_connect = on_connect
        self.mqtt_client.on_publish = on_publish
        
        ## Make the connection and set it up persistent
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

    def mqtt_callback(self, msg) -> None:
        """
        Callback that read the stamp in the value and obtain flag status, then
        send it to Thingsboard.

        Param
        ---
        msg : std_msgs.msg.Bool
            Acceleration message received
        """
        # Assgin components to the dict
        self.sensor_data['fall'] = msg.data

        # Publish it to the specified topic
        self.mqtt_client.publish(self.MQTT_TOPIC, json.dumps(self.sensor_data), 1)


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
    Program that receives info of the accelerometer topic and sent it to the
    Thinkspeak platform where a the broker is located.
    """
    # Initialize node
    rclpy.init(args=args)

    # Instance subscriber
    fall_to_mqtt = ThingsboardFallSubs()

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
