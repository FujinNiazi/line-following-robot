#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.bridge = CvBridge()
        
        # Subscriber to the camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # Publisher to command velocity topic
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info("Line follower node started")

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Use only the bottom part of the image
        height, width = frame.shape[:2]
        roi = gray[int(0.7*height):, :]  # Lower 30% of the image
        
        # Threshold image to get the line (adjust values as needed)
        _, thresh = cv2.threshold(roi, 100, 255, cv2.THRESH_BINARY_INV)
        
        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find largest contour assuming it's the line
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Calculate moments to find centroid
            M = cv2.moments(largest_contour)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                
                # Calculate error from center of image
                error = cx - (frame.shape[1] // 2)
                
                # Create Twist message
                twist = Twist()
                twist.linear.x = 0.2  # constant forward speed
                Kp = 0.002  # Smaller gain
                twist.angular.z = -Kp * float(error)
                max_ang = 0.5 
                twist.angular.z = max(min(twist.angular.z, max_ang), -max_ang) # Clamp the max 
                
                self.publisher_.publish(twist)
                
                self.get_logger().info(f"Error: {error}, Steering: {twist.angular.z:.2f}")
            else:
                self.get_logger().info("No line detected - stopping")
                self.publisher_.publish(Twist())  # stop
        else:
            twist = Twist()
            twist.angular.z = 0.2  # slow rotation to find line
            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
