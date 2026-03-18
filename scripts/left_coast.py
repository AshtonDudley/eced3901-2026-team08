#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
# Updated March 12, 2026 - By: Megan Neville
# Navigation 2 Waypoint Follower with Sensor Interrupt Override

from copy import deepcopy
import math
import time
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

# --- Helper Functions ---

def get_quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

# --- Sensor Monitor Node ---

class SensorMonitor(Node):
    def __init__(self):
        super().__init__('sensor_monitor')
        self.lidar_front = None
        self.ultrasonic_front = None
        self.lidar_angle_idx = None
        self.amcl_pose = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, sensor_qos)
        self.ultrasonic_sub = self.create_subscription(LaserScan, '/ultrasonic_scan', self.ultrasonic_callback, sensor_qos)
        
        # Subscribe to AMCL to get the true map location
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

        # Publisher for Manual Motor Override
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def pose_callback(self, msg):
        self.amcl_pose = msg.pose.pose

    def lidar_callback(self, msg):
        if self.lidar_angle_idx is None:
            if msg.angle_increment == 0:
                return
            idx = int((0.0 - msg.angle_min) / msg.angle_increment)
            if 0 <= idx < len(msg.ranges):
                self.lidar_angle_idx = idx
                
        if self.lidar_angle_idx is not None and self.lidar_angle_idx < len(msg.ranges):
            self.lidar_front = msg.ranges[self.lidar_angle_idx]

    def ultrasonic_callback(self, msg):
        if msg.ranges:
            self.ultrasonic_front = msg.ranges[0]

    def difference(self, threshold):
        if self.lidar_front is None or self.ultrasonic_front is None:
            return False
            
        if math.isinf(self.lidar_front) or math.isinf(self.ultrasonic_front):
            return False
            
        diff = self.lidar_front - self.ultrasonic_front
        if diff > threshold:
            self.get_logger().warn(f"*** INTERRUPT TRIGGERED! Lidar: {self.lidar_front:.2f}m | US: {self.ultrasonic_front:.2f}m | Diff: {diff:.2f}m ***")
            return True
            
        return False

# --- Main Navigation Loop ---

def main():
    DIFF_THRESHOLD = 0.15
    TARGET_DISTANCE = 0.05 # Stopping 5cm away from the object
    MANUAL_SPEED = 0.05    # Move at 5 cm/s during manual override
    COOLDOWN_TIME = 10.0   # Prevent back-to-back rapid interrupts
    last_interrupt_time = None
    interrupted_waypoint_idx = None

    rclpy.init()

    sensor_monitor = SensorMonitor()
    navigator = BasicNavigator()

    inspection_route = [
        [1.8, 0.6, 0.78],
        [2.6, 0.1, 0.0],
        [3.85, 0.6, 1.9],
        [2.8, 0.3, 3.14],
        [3.45, -0.2, 3.20],
        [1.8, 0.6, 3.14],
        [-0.5, -0.5, 0.0]
    ]

    navigator.waitUntilNav2Active()

    inspection_points = []
    for pt in inspection_route:    
        inspection_pose = PoseStamped()
        inspection_pose.header.frame_id = 'map'
        inspection_pose.header.stamp = navigator.get_clock().now().to_msg()
        inspection_pose.pose.position.x = pt[0]
        inspection_pose.pose.position.y = pt[1]
        q = get_quaternion_from_euler(0,0,pt[2])
        inspection_pose.pose.orientation.x = q[0]
        inspection_pose.pose.orientation.y = q[1]      
        inspection_pose.pose.orientation.z = q[2]
        inspection_pose.pose.orientation.w = q[3]  
        inspection_points.append(deepcopy(inspection_pose))
        
    navigator.followWaypoints(inspection_points)

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        rclpy.spin_once(sensor_monitor, timeout_sec=0.1)
        feedback = navigator.getFeedback()
        
        if feedback and i % 5 == 0:
            print(f"Executing waypoint: {feedback.current_waypoint + 1}/{len(inspection_points)}")

        now = time.time()
        
        # Grab true map coordinates from AMCL subscriber
        xpose = sensor_monitor.amcl_pose.position.x if sensor_monitor.amcl_pose else 0.0
        ypose = sensor_monitor.amcl_pose.position.y if sensor_monitor.amcl_pose else 0.0

        if last_interrupt_time is None or (now - last_interrupt_time) > COOLDOWN_TIME:
            if sensor_monitor.difference(DIFF_THRESHOLD) and not (ypose > 3.0 and xpose < 0.3):
                
                print('\n>>> Halting Nav2 for Inspection Maneuver! <<<\n')
                last_interrupt_time = time.time()
                
                if feedback:
                    interrupted_waypoint_idx = feedback.current_waypoint
                else:
                    interrupted_waypoint_idx = 0

                # 1. Cancel Nav2 task so it releases control of the motors
                navigator.cancelTask()
                while not navigator.isTaskComplete():
                    rclpy.spin_once(sensor_monitor, timeout_sec=0.1)

                print('>>> Driving forward manually... <<<')
                
                # 2. Manual Motor Override: Inch forward until close
                twist_msg = Twist()
                twist_msg.linear.x = MANUAL_SPEED 
                
                start_drive = time.time()
                # Stop if we hit target, or timeout after 10 seconds just in case
                while (sensor_monitor.ultrasonic_front is not None and 
                       sensor_monitor.ultrasonic_front > TARGET_DISTANCE and
                       (time.time() - start_drive) < 10.0):
                    sensor_monitor.cmd_vel_pub.publish(twist_msg)
                    rclpy.spin_once(sensor_monitor, timeout_sec=0.1)
                    
                # 3. Stop Forward Motion
                twist_msg.linear.x = 0.0
                sensor_monitor.cmd_vel_pub.publish(twist_msg)

                print('>>> Rotating 360 degrees... <<<')
                
                # 4. Manual Motor Override: Rotate in place
                twist_msg.angular.z = 0.5 # Rad/s speed
                start_rotate = time.time()
                rotate_duration = (2.0 * math.pi) / twist_msg.angular.z
                
                while (time.time() - start_rotate) < rotate_duration:
                    sensor_monitor.cmd_vel_pub.publish(twist_msg)
                    rclpy.spin_once(sensor_monitor, timeout_sec=0.1)
                    
                # 5. Stop Rotation
                twist_msg.angular.z = 0.0
                sensor_monitor.cmd_vel_pub.publish(twist_msg)

                print('>>> Maneuver complete. Resuming waypoints... <<<\n')

                # 6. Give control back to Nav2
                if interrupted_waypoint_idx is not None:
                    remaining = inspection_points[interrupted_waypoint_idx:]
                    if remaining:
                        navigator.followWaypoints(remaining)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Inspection complete! Returning to start...')
    elif result == TaskResult.CANCELED:
        print('Inspection was canceled.')
    elif result == TaskResult.FAILED:
        print('Inspection failed!')

    sensor_monitor.destroy_node()
    rclpy.shutdown()
    exit(0)

if __name__ == '__main__':
    main()