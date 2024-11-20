import json
import paho.mqtt.client as mqtt
from g_mov_msgs.msg import ServoPoseStamped
import rclpy
from rclpy.node import Node

# Global variables
servo_value = 0
send_flag = False
sensor_data = {'Servomotor_Angle': 25}

class ThingsBoardToCmdVel(Node):
    def __init__(self):
        super().__init__('thingsboard_to_cmd_vel')
        self.publisher = self.create_publisher(ServoPoseStamped, '/servo_angle', 10)
        self.timer = self.create_timer(4, self.timer_callback)
        
        # MQTT configuration
        self.client = mqtt.Client()
        self.BROKER = 'mqtt.local'
        self.PORT = 1883
        self.ACCESS_TOKEN = '2gImYrCCe2j29PRQcij8'
        
        # MQTT callbacks
        self.client.on_connect = self.on_connect  
        self.client.on_message = self.on_message
        
        self.client.username_pw_set(self.ACCESS_TOKEN)
        self.client.connect(self.BROKER, self.PORT, 60)
        self.client.subscribe('v1/devices/me/rpc/request/+')
        self.client.loop_start()

    def send_velocity(self, grad: float):
        """Publishes a velocity command."""
        msg = ServoPoseStamped()
        msg.data = grad
        self.publisher.publish(msg)

    def shutdown(self):
        """Stops MQTT loop and disconnects."""
        self.client.loop_stop()
        self.client.disconnect()

    def timer_callback(self):
        """Periodically sends velocity if the flag is set."""
        global send_flag, servo_value
        if send_flag:
            msg = ServoPoseStamped()
            msg.data = servo_value
            self.publisher.publish(msg)
            send_flag = False  # Reset flag after sending

    def on_connect(self, client, userdata, flags, rc):
        """Callback when connected to MQTT broker."""
        if rc == 0:
            self.get_logger().info("Connected to MQTT broker.")
            client.subscribe('v1/devices/me/rpc/request/+')
        else:
            self.get_logger().error(f"Failed to connect, return code {rc}")

    def on_message(self, client, userdata, msg):
        """Callback when an MQTT message is received."""
        global send_flag, servo_value, sensor_data
        if msg.topic.startswith('v1/devices/me/rpc/request/'):
            try:
                requestId = msg.topic[len('v1/devices/me/rpc/request/'):len(msg.topic)]
                data = json.loads(msg.payload.decode())
                if data.get('method') == 'setValue':
                    params = data.get('params', 0)
                    servo_value = float(params)
                    sensor_data['ServoMotor_Angle'] = params
                    send_flag = True
                    self.get_logger().info(f"Received setValue: {params}")
                if data['method'] == 'getValue':
                    #print("getvalue request\n")
                    #print("sent getValue : ", sensor_data)
                    client.publish('v1/devices/me/rpc/response/' + requestId, json.dumps(sensor_data['ServoMotor_Angle']), 1)
            except json.JSONDecodeError as e:
                self.get_logger().error(f"Error decoding message: {e}")

def main():
    """Main entry point for the ROS2 node."""
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
