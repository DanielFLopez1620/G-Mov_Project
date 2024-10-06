#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ----------------------- PYTHON REQUIRED LIBRARIES ---------------------------
import paho.mqtt.publish as publish
import cv2
import time
import base64

# ------------------------- ROS2 REQUIRED LIBRARIES ---------------------------
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

# ------------------------ ROS2 REQUIRED MESSAGES -----------------------------
from sensor_msgs.msg import Image

# ---------------------- CAM SUBSCRIBER TO THINGSPEAK PUBLISHER ---------------
class ThingSpeakCamSubs(Node):
    """
    Class that subscriber to the camera topic (raw image), obtain the data, 
    transform it to text and then publish it to the ThingSpeak platform where is
    located a MQTT broker with a specific format that consider timeStamp, 
    proper sensor and variable ID, the value (in this case encoded in base 64 to
    text) and the measurament (which specifies that comes from encoding).
    """
    def __init__(self):
        """
        Constructor that sets up the communication by using MQTT with paho, to
        connect with the Thinkspeak broker as it provides the proper validation
        to the topic with the user and password generated. It also initialize
        the node with the name "thingspeak_cam_sub", create the proper 
        susbcriber with a callback for interpreting the image and initialize
        some of the mqtt params and conversion image params.
        """
        # Initialize node
        super().__init__('thingspeak_cam_sub')

        # Set thingspeak params for MQTT connection
        self.channel_id = "2667301"
        self.mqtt_client_ID = "ER0GJDQkIQsBNR0hGjcqIyw"
        self.mqtt_username = "ER0GJDQkIQsBNR0hGjcqIyw"
        self.mqtt_password = "Fu4VIaTN2ZtaoqTuMUnGYv5D"
        self.mqtt_topic = "channels/" + self.channel_id + "/publish"
        self.mqtt_host = "mqtt3.thingspeak.com"
        
        # Initialize values for MQTT tranmission
        self.id_sensor = 3
        self.variable_sensor = 1
        self.timestamp = 0
        self.unit_sensor = "jpg format"
        self.payload = ""

        # Subscribing to /image_raw
        self.subscription = self.create_subscription(Image, '/image_raw', self.mqttt_callback, 10)

        # For image conversion
        self.bridge = CvBridge()
        self.publish_rate = 1.0 / 1.0  # 1 FPS
        self.last_publish_time = time.time()

    def mqttt_callback(self, msg):
        """
        Callback that read the stamp and the float value of the servo, then
        it publish to MQTT.
        """
        self.timestamp = msg.header.stamp

        # Limit publishing rate to 1 FPS
        current_time = time.time()
        if current_time - self.last_publish_time < self.publish_rate:
            return

        # Convert ROS2 image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Convert the image to a base64 string
        _, buffer = cv2.imencode('.jpg', cv_image)
        jpg_as_text = base64.b64encode(buffer).decode('utf-8')
        
        # Update payload with the proper fields order
        self.payload = "field1=" + str(self.id_sensor) 
        self.payload += "&field2=" + str(self.timestamp) 
        self.payload += "&field3=" + str(self.variable_sensor) 
        self.payload += "&field4=" + str(jpg_as_text) 
        self.payload += "&field5=" + str(self.unit_sensor)

        # Publish to MQTT
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

        # Update values before cycle start again
        self.payload = ""
        self.last_publish_time = current_time

# ----------------------- MAIN IMPLEMENTATION ---------------------------------
def main(args=None):
    """
    Program that receives the raw image of the camara, process it and send it to
    the Thingspeak platform where the broker is located.
    """
    # Initialize ndoe
    rclpy.init(args=args)

    # Instance subscriber
    image_mqtt = ThingSpeakCamSubs()

    # Spin node
    rclpy.spin(image_mqtt)

    # Clean and shutdown
    image_mqtt.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
