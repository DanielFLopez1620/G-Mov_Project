import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from g_mov_msgs.msg import ServoPoseStamped # Assuming custom message with a header and angle data

class HeadTracker(Node):
    def __init__(self):
        super().__init__('head_tracker')
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Create subscriber for the image
        self.image_subscriber = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)
        
        # Create a publisher for the servo angle
        self.servo_pub = self.create_publisher(ServoPoseStamped, '/servo_angle', 10)
        
        # Initialize head detection (using Haar Cascades)
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        
        # Parameters for the servo
        self.servo_angle = 90  # Start with the camera facing forward (horizontal)
        self.servo_max = 180  # Max upward angle
        self.servo_min = 0    # Min downward angle
        self.servo_step = 2   # Servo adjustment step (degrees per update)
        
        # Store image height to calculate offset from center
        self.image_height = 480  # Assume 480p resolution for now
    
    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_image_rev = cv2.flip(cv_image, 0)
        gray = cv2.cvtColor(cv_image_rev, cv2.COLOR_BGR2GRAY)

        # Detect faces in the image
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)

        # If faces are detected
        if len(faces) > 0:
            # Assume we track the first detected face
            (x, y, w, h) = faces[0]
            face_center_y = y + h // 2
            
            # Determine the offset from the center of the frame
            frame_center_y = self.image_height // 2
            offset = face_center_y - frame_center_y
            
            # Move the servo based on the offset
            self.adjust_servo_angle(offset)
        
        # Display the image with the detected face (for visualization/debugging)
        for (x, y, w, h) in faces:
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
        
        cv2.imshow("Head Tracker", cv_image)
        cv2.waitKey(1)
    
    def adjust_servo_angle(self, offset):
        threshold = 20  # Pixel threshold for movement (tune this for sensitivity)

        # If the face is above the center, decrease the servo angle (move up)
        if offset < -threshold and self.servo_angle < self.servo_max:
            self.servo_angle += self.servo_step
        # If the face is below the center, increase the servo angle (move down)
        elif offset > threshold and self.servo_angle > self.servo_min:
            self.servo_angle -= self.servo_step
        
        # Publish the updated servo angle
        servo_msg = ServoPoseStamped()
        servo_msg.header.stamp = self.get_clock().now().to_msg()  # Add timestamp
        servo_msg.data = float(self.servo_angle)
        self.servo_pub.publish(servo_msg)
        self.get_logger().info(f"Servo angle updated: {self.servo_angle}°")

def main(args=None):
    rclpy.init(args=args)
    head_tracker = HeadTracker()
    rclpy.spin(head_tracker)
    head_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
