import json
import paho.mqtt.client as mqtt
from g_mov_msgs.msg import ServoPoseStamped
import rclpy
from rclpy.node import Node

BROKER = 'mqtt.local'  # ThingsBoard IP
PORT = 1883
ACCESS_TOKEN = '2gImYrCCe2j29PRQcij8'  # Replace with the device access token

class ThingsBoardToCmdVel(Node):
    def __init__(self):
        super().__init__('thingsboard_to_cmd_vel')
        self.publisher = self.create_publisher(ServoPoseStamped, '/cmd_vel', 10)
        self.client = mqtt.Client()
        self.client.on_message = self.on_message
        self.client.username_pw_set(ACCESS_TOKEN)
        self.client.connect(BROKER, PORT, 60)
        self.client.subscribe('v1/devices/me/DanEsteNoEsElAngulo')
        self.client.loop_start()

    def on_message(self, client, userdata, msg):
        payload = json.loads(msg.payload.decode('utf-8'))
        method = payload.get('method')
        self.send_velocity(method)

    def send_velocity(self, grad):
        msg = ServoStamped()
        msg.data = grad
        self.publisher.publish(msg)

    def shutdown(self):
        self.client.loop_stop()
        self.client.disconnect()

def main():
    rclpy.init()
    node = ThingsBoardToCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
