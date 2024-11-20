#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# -------------------------- PYTHON REQUIRED LIBRARIES ------------------------
import paho.mqtt.client as mqtt   # Library for MQTT connections
import json                       # Json formatting 

# ------------------------- ROS 2 REQUIRED LIBRARIES --------------------------
import rclpy                     # ROS 2 Client Library for Python 
from rclpy.node import Node      # Base class for nodes

# --------------------------- ROS 2 REQUIRED MESSAGES -------------------------
from g_mov_msgs.msg import ServoPoseStamped # Custom servo msg

# --------------------------- GLOBAL VARIABLES --------------------------------
# I know, it is not recommeded but Thingsboard communications got some problems
# and this was the quick fix
servo_value = 0
send_flag = False
sensor_data = {'Servomotor_Angle': 25}

# ------------------ SERVO ROS2 PUBLISHER BASED ON THINGSPEAK VALUE  ----------
class ThingsBoardToServo(Node):
    """
    Class that subscriber to the servo topic controlled in the dashboard in the
    Thingsboard platform, obtain the data and then publish it (by using a 
    ROS 2 publisher) to the servo, in order to change the position.
    """
    def __init__(self):
        """
        Constructor that sets up the communication by using MQTT with paho, to
        connect with the Thingsboard broker as it provides the proper validation
        to the topic with the user and password generated. It also initialize
        the node with the name "thingsboard_to_servo", create the proper publisher
        based on the info received on Thingsboard, with a timer callback for 
        interpreting the values and send them.
        """
        # Initialize node
        super().__init__('thingsboard_to_servo')

        # Instantiate publisher for the servo
        self.publisher = self.create_publisher(ServoPoseStamped, '/servo_angle', 10)

        # Create a timer that validates each two second if there are changes
        self.timer = self.create_timer(2, self.timer_callback)
        
        # MQTT configuration
        self.client = mqtt.Client()
        self.BROKER = 'mqtt.local'
        self.PORT = 1883
        self.ACCESS_TOKEN = '2gImYrCCe2j29PRQcij8'
        
        # MQTT callbacks
        self.client.on_connect = self.on_connect  
        self.client.on_message = self.on_message
        
        # Set connection for Thingsboard broker
        self.client.username_pw_set(self.ACCESS_TOKEN)
        self.client.connect(self.BROKER, self.PORT, keepalive=60)

        # Subscribe an make it persistent
        self.client.subscribe('v1/devices/me/rpc/request/+')
        self.client.loop_start()

    def send_velocity(self, grad) -> None:
        """
        Publishes the servo angle by using the custom g_mov_msgs

        Params
        ---
        grad : float
            New servo pose objective
        """
        msg = ServoPoseStamped()
        msg.data = grad
        self.publisher.publish(msg)

    def shutdown(self) -> None:
        """
        Stops MQTT loop and disconnects from the Thingsboard broker
        """
        self.client.loop_stop()
        self.client.disconnect()

    def timer_callback(self) -> None:
        """
        If the flag is true (indicates a change in Thingsboard's dashboard),
        then update the servo position.
        """
        # Use of global variable
        global send_flag, servo_value
        
        # Validation
        if send_flag:
            # Update servo value and add time stamp
            msg = ServoPoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.data = servo_value

            # Publish and reset flag
            self.publisher.publish(msg)
            send_flag = False  

    def on_connect(self, client, userdata, flags, rc):
        """
        On Mqtt connecction, notify and create a log to specify if it was
        succesful or returned an error code. Also, it tries again the
        connection (subscribe)

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
            self.get_logger().info("Connected to MQTT broker.")
            client.subscribe('v1/devices/me/rpc/request/+')
        else:
            self.get_logger().error(f"Failed to connect, return code {rc}")

    def on_message(self, client, userdata, msg) -> None:
        """
        If a message is received, process the servo value, also notifies the
        change in thingsboard's dashboard.

        Params
        ---
        client : paho.mqtt.client.Client
            Instance client for the callback
        
        userdata : paho.mqtt.client.Client.user_data_set()
            User defined data in client

        msg : paho.mqtt.client.MQTTMessage
            Message received during connection with the broker
        """
        # Use global variables
        global send_flag, servo_value, sensor_data

        # Check topic
        if msg.topic.startswith('v1/devices/me/rpc/request/'):
            # In case of JSON errors
            try:
                requestId = msg.topic[len('v1/devices/me/rpc/request/'):len(msg.topic)]
                # Decode data in JSon format
                data = json.loads(msg.payload.decode())

                # If it is the set topic... update value and raise flag
                if data.get('method') == 'setValue':
                    params = data.get('params', 0)
                    servo_value = float(params)
                    sensor_data['ServoMotor_Angle'] = params
                    send_flag = True
                    self.get_logger().info(f"Received setValue: {params}")
                # If it is the get value, update with the current pose
                if data['method'] == 'getValue':
                    client.publish('v1/devices/me/rpc/response/' + requestId, 
                        json.dumps(sensor_data['ServoMotor_Angle']), 1)
            except json.JSONDecodeError as e:
                self.get_logger().error(f"Error decoding message: {e}")

        def __del__(self):
            """
            Destruction that will kill connections
            """
            self.shutdown()

def main():
    """
    Node oriented for servo interaction with dashboard indicator on Thingsboard
    platform
    """
    # Initialize node
    rclpy.init()

    # Instantiate class
    mqtt_to_servo = ThingsBoardToServo()

    # Manage keyboard interrupt
    try:
        rclpy.spin(mqtt_to_servo)
    except KeyboardInterrupt:
        mqtt_to_servo.shutdown()
    finally:
        # Clean and shutdown
        mqtt_to_servo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
