#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# -------------------------- PYTHON REQUIRED LIBRARIES ------------------------
import paho.mqtt.client as mqtt   # Library for MQTT connections
import ssl                        # Related with secure TLS

# ------------------------- ROS 2 REQUIRED LIBRARIES --------------------------
import rclpy                     # ROS 2 Client Library for Python 
from rclpy.node import Node      # Base class for nodes

# --------------------------- ROS 2 REQUIRED MESSAGES --------------------------
from geometry_msgs.msg import AccelStamped  # Acceleration msg (linear/angular)

# -------------------- ACCEL SUBSCRIBER TO MOSQUITTO PUBLISHER ---------------
class MosquittoAccelSubs(Node):
    """
    Class that subscriber to the accelerometer topic, obtain the data and then
    publish it to the Mosquitto local broker with a specific format that 
    consider timeStamp, proper sensor and variable ID, the value and the 
    measurament.
    """
    def __init__(self):
        """
        Constructor that sets up the communication by using MQTT with paho, to
        connect with the Mosquitto broker as it provides the proper validation
        to the topic with the user and password generated. It also initialize
        the node with the name "mosquitto_accel_sub", create the proper susbcriber
        with a callback for interpreting the values and initialize some of the
        mqttt params.
        """
        # Initialize node
        super().__init__('mosquitto_accel_sub')

        # Set Mosquitto params for MQTT with TLS connection over mosquitto
        self.MQTT_BROKER = "mqtt.local"
        self.MQTT_PORT = 8883
        self.MQTT_TOPIC = "sensors/accel"
        self.MQTT_CLIENT_ID = "mqtt_pub_accel_client"
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
        self.mqtt_client.connect(self.MQTT_BROKER, port=self.MQTT_PORT, keepalive=60)
        self.mqtt_client.loop_start()

        # Initialize values for MQTT transmission
        self.ID_SENSOR = 2         # Sensor id for accelerometer is 2
        self.timestamp = 0         # Initialize void stamp
        self.UNIT_SENSOR = "m/s²"  # Acceleration units considered

        # Instance suscription with the callback that will send info to ThingSp
        self.subs = self.create_subscription(AccelStamped, 'accel_info', 
            self.mqtt_callback, 10)

    def __del__(self):
        """
        Destructor to clean up MQTT connections to the Mosquitto broker.
        """
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()

    def mqtt_callback(self, msg) -> None:
        """
        Callback that read the stamp in the value and classifies each linear
        component received to create 6 publication (one for component) that are
        sent to Mosquitto.

        Params
        ---
        msg : geometry_msgs.msg.AccelStamped
            Message received from accelerometer node 
        """
        # Get stamp
        self.timestamp = msg.header.stamp

        # Obtain components
        accel_val= (msg.accel.linear.x, msg.accel.linear.y, msg.accel.linear.z,
            msg.accel.angular.x, msg.accel.angular.y, msg.accel.angular.z)
        
        # Counter will specify sensor value Id, in this case they are:
        # - 1 (Linear x), 2 (Linear y), 3 (Linear z)
        # - 4 (Angular x), 5 (Angular y), 6 (Angular z)
        counter = 1

        # Iterate to publish each component
        for val in accel_val:
            # Update payload with the proper fields order
            payload = "field1=" + str(self.ID_SENSOR) 
            payload += "&field2=" + str(self.timestamp) 
            payload += "&field3=" + str(counter) 
            payload += "&field4=" + str(val) 
            payload += "&field5=" + str(self.UNIT_SENSOR)

            # Publish to MQTT
            flag = self.mqtt_client.publish(self.MQTT_TOPIC, payload)
            flag.wait_for_publish()

            # Update values before cycle start again
            counter += 1

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
    Program that receives info of the accelerometer topic and sent it to the
    Mosquitto local broker.
    """
    # Initialize node
    rclpy.init(args=args)

    # Instance subscriber
    accel_to_mqtt = MosquittoAccelSubs()

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
