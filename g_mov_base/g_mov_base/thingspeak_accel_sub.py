#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import paho.mqtt.publish as publish

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import AccelStamped

class ThingSpeakAccelSubs(Node):
    def __init__(self):
        super().__init__('thingspeak_accel_subs')
        self.channel_id = "2667301"
        self.mqtt_client_ID = "ER0GJDQkIQsBNR0hGjcqIyw"
        self.mqtt_username = "ER0GJDQkIQsBNR0hGjcqIyw"
        self.mqtt_password = "Fu4VIaTN2ZtaoqTuMUnGYv5D"
        self.mqtt_topic = "channels/" + self.channel_id + "/publish"
        self.mqtt_host = "mqtt3.thingspeak.com"
        self.id_sensor = 2
        self.timestamp = 0
        self.unit_sensor = "m/s²"
        self.payload = ""
        self.subs = self.create_subscription(AccelStamped, 'accel_info', self.mqtt_callback, 10)

    def __del__(self):
        pass
        
    def mqtt_callback(self, msg):
        self.timestamp = msg.header.stamp
        accel_val= (msg.accel.linear.x, msg.accel.linear.y, msg.accel.linear.z)
        counter = 1
        for val in accel_val:
            self.payload = "field1=" + str(self.id_sensor) 
            self.payload += "&field2=" + str(self.timestamp) 
            self.payload += "&field3=" + str(counter) 
            self.payload += "&field4=" + str(val) 
            self.payload += "&field5=" + str(self.unit_sensor)
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
            counter += 1
            self.payload = ""


def main(args=None):
    rclpy.init(args=args)
    accel_to_mqtt = ThingSpeakAccelSubs()
    rclpy.spin(accel_to_mqtt)
    rclpy.shutdown()   


if __name__ == "__main__":
    main()