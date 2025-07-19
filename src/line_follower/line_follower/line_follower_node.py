#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import time



class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.bridge = CvBridge()

        # Subscribing to image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

        # Publishing to /cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Recent turning history to infer recovery direction
        self.recent_turns = deque(maxlen=50)
        self.recovery_direction = 0.3  # default fallback
        
        # Position tracking
        self.initial_pose = None
        self.position_threshold = 0.1  # how close to stop
        self.activation_distance = 0.5  # how far to go before checking for return
        self.moved_away = False
        
        self.sampled_line_color = None  # HSV color of the line
        self.color_sampled = False


        self.get_logger().info("🚗 Line follower node started")


    def image_callback(self, msg):
        twist = Twist()

        # Convert to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        height, width = hsv.shape[:2]
        
        roi = hsv[int(0.5 * height):, :] 

        # Step 1: Sample bottom center color (just once)
        if not self.color_sampled:
            center_sample = hsv[-20:, width // 2 - 10: width // 2 + 10]
            self.sampled_line_color = np.mean(center_sample.reshape(-1, 3), axis=0)
            self.color_sampled = True
            self.get_logger().info(f"🎯 Sampled line HSV color: {self.sampled_line_color}")

        # Step 2: Threshold by color match instead of grayscale
        lower = np.clip(self.sampled_line_color - np.array([10, 60, 60]), 0, 255)
        upper = np.clip(self.sampled_line_color + np.array([10, 80, 80]), 0, 255)
        mask = cv2.inRange(roi, lower.astype(np.uint8), upper.astype(np.uint8))

        # Contour detection
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Found line — follow it
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            
            # Compute centroid

            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                error = cx - (width // 2)

                # Steering control
                Kp = 0.002
                angular_z = -Kp * float(error)
                max_ang = 0.5
                angular_z = max(min(angular_z, max_ang), -max_ang)
                twist.angular.z = angular_z

                # Slow down if turning sharply
                turn_strength = abs(angular_z)
                base = 0.3 # base (max) speed when going straight
                turn = 0.4 # how aggressively speed drops when turning
                min_speed = 0.1 # minimum speed 
                twist.linear.x = max(min_speed, base - turn * turn_strength)

                # Store recent turning direction for recovery
                self.recent_turns.append(angular_z)
                if len(self.recent_turns) >= 5:
                    signs = [np.sign(val) for val in self.recent_turns]
                    count_pos = signs.count(1)
                    count_neg = signs.count(-1)

                    confidence = max(count_pos, count_neg) / len(signs)

                    if confidence >= 0.9:
                        avg_turn = sum(self.recent_turns) / len(self.recent_turns)
                        if abs(avg_turn) >= 0.07 and abs(avg_turn - self.recovery_direction) >= 0.07:
                            alpha = 0.2
                            self.recovery_direction = (1 - alpha) * self.recovery_direction + alpha * avg_turn
                            
                            self.get_logger().info(
                                f"💾 Updated recovery_direction (conf {confidence:.2f}): {self.recovery_direction:.2f}")
                            self.get_logger().info(f"✅ Line detected — Error: {error}, Turning: {angular_z:.2f}")

            else:
                contours = None                     
                                
        else:
            # Line lost: recover by rotating in last known direction
            if self.check_for_road_pattern(hsv):
                self.get_logger().info("🛣️ Road pattern detected — updating line color")
            
            else:
                self.get_logger().info("❌ No road-like pattern found — color not updated")    
                twist.linear.x = 0.0
                twist.angular.z = np.sign(self.recovery_direction) * 0.5
                self.get_logger().warn("🔄 Lost line — rotating in last known direction (from contour trend)")

        self.publisher_.publish(twist)
        
    def check_for_road_pattern(self, hsv_frame):
        height, width = hsv_frame.shape[:2]
        roi = hsv_frame[int(0.5 * height):, :]

        # Sample 3 zones: left, center, right
        left = roi[:, :width // 3]
        center = roi[:, width // 3: 2 * width // 3]
        right = roi[:, 2 * width // 3:]

        # Get average HSV
        avg_left = np.mean(left.reshape(-1, 3), axis=0)
        avg_center = np.mean(center.reshape(-1, 3), axis=0)
        avg_right = np.mean(right.reshape(-1, 3), axis=0)

        # Check if left and right are similar, but center is different
        lr_diff = np.linalg.norm(avg_left - avg_right)
        lc_diff = np.linalg.norm(avg_left - avg_center)
        rc_diff = np.linalg.norm(avg_right - avg_center)

        if lr_diff < 30 and lc_diff > 40 and rc_diff > 40:
            # Likely a different line in center
            self.sampled_line_color = avg_center
            self.color_sampled = True
            return True
        return False

        
    def odom_callback(self, msg):
        pos = msg.pose.pose.position

        if self.initial_pose is None:
            self.initial_pose = pos
            self.get_logger().info(f"📍 Initial position recorded: x={pos.x:.2f}, y={pos.y:.2f}")
            return

        dx = pos.x - self.initial_pose.x
        dy = pos.y - self.initial_pose.y
        dist = (dx**2 + dy**2)**0.5

        # Wait until robot has moved far enough before enabling stop check
        if not self.moved_away and dist > self.activation_distance:
            self.moved_away = True
            self.get_logger().info("🏁 Robot moved away from start — return detection activated.")
            return

        if self.moved_away and dist < self.position_threshold:
            self.get_logger().info("🛑 Back to initial position — stopping robot.")
            self.publisher_.publish(Twist())
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
