#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Updated March 12, 2026
# By: Megan Neville

from copy import deepcopy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
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

def get_yaw_from_quaternion(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

# --- Sensor Monitor Node ---

class SensorMonitor(Node):
    def __init__(self):
        super().__init__('sensor_monitor')
        self.lidar_front = None
        self.ultrasonic_front = None
        self.lidar_angle_idx = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, sensor_qos)
        self.ultrasonic_sub = self.create_subscription(LaserScan, '/ultrasonic_scan', self.ultrasonic_callback, sensor_qos)

    def lidar_callback(self, msg):
        if self.lidar_angle_idx is None:
            if msg.angle_increment == 0:
                return
            idx = int((0.0 - msg.angle_min) / msg.angle_increment)
            if 0 <= idx < len(msg.ranges):
                self.lidar_angle_idx = idx
            else:
                self.get_logger().warn("Lidar angle index out of range")
                return
                
        if self.lidar_angle_idx is not None and self.lidar_angle_idx < len(msg.ranges):
            self.lidar_front = msg.ranges[self.lidar_angle_idx]

    def ultrasonic_callback(self, msg):
        if msg.ranges:
            self.ultrasonic_front = msg.ranges[0]

    def difference(self, threshold):
        if self.lidar_front is None or self.ultrasonic_front is None:
            return False
        # Filter out invalid math from infinite distance readings
        if math.isinf(self.lidar_front) or math.isinf(self.ultrasonic_front):
            return False
            
        if self.lidar_front - self.ultrasonic_front < 0:
            return False
        return (self.lidar_front - self.ultrasonic_front) > threshold


# --- Main Navigation Loop ---

def main():
    # Define Params
    DIFF_THRESHOLD = 0.15
    TARGET_DISTANCE = 0.01
    STEP_SIZE = 0.05
    COOLDOWN_TIME = 1.0
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
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(inspection_points)))

        now = navigator.get_clock().now().seconds_nanoseconds()[0]
        
        # Grab current position for the exclusion zone check
        current_pose = navigator.getCurrentPose()
        xpose = current_pose.pose.position.x if current_pose else 0.0
        ypose = current_pose.pose.position.y if current_pose else 0.0

        if last_interrupt_time is None or (now - last_interrupt_time) > COOLDOWN_TIME:
            if sensor_monitor.difference(DIFF_THRESHOLD) and not (ypose > 3.0 and xpose < 0.3):
                print('\n>>> Interrupt triggered! Difference large. <<<\n')
                last_interrupt_time = now
                if feedback:
                    interrupted_waypoint_idx = feedback.current_waypoint
                else:
                    interrupted_waypoint_idx = 0

                navigator.cancelTask()
                while not navigator.isTaskComplete():
                    rclpy.spin_once(sensor_monitor, timeout_sec=0.1)
                    
                result = navigator.getResult()
                print(f'Cancelled with result: {result}')

                if current_pose is None:
                    print('Failed to get current pose – skipping interrupt routine')
                else:
                    steps = 0
                    max_steps = 10
                    while (sensor_monitor.ultrasonic_front is not None and sensor_monitor.ultrasonic_front > TARGET_DISTANCE and steps < max_steps):
                        yaw = get_yaw_from_quaternion(current_pose.pose.orientation)
                        new_x = current_pose.pose.position.x + STEP_SIZE * math.cos(yaw)
                        new_y = current_pose.pose.position.y + STEP_SIZE * math.sin(yaw)

                        goal_pose = PoseStamped()
                        goal_pose.header.frame_id = 'map'
                        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
                        goal_pose.pose.position.x = new_x
                        goal_pose.pose.position.y = new_y
                        goal_pose.pose.orientation = current_pose.pose.orientation

                        navigator.goToPose(goal_pose)

                        while not navigator.isTaskComplete():
                            rclpy.spin_once(sensor_monitor, timeout_sec=0.1)
                            if (sensor_monitor.ultrasonic_front is not None and sensor_monitor.ultrasonic_front <= TARGET_DISTANCE):
                                navigator.cancelTask()
                                break

                        current_pose = navigator.getCurrentPose()
                        steps += 1

                    # Rotate ~180 degrees twice to ensure Nav2 actually spins
                    if current_pose is not None:
                        for rotation_offset in [math.pi, 2.0 * math.pi]:
                            yaw = get_yaw_from_quaternion(current_pose.pose.orientation)
                            new_yaw = yaw + rotation_offset
                            q = get_quaternion_from_euler(0, 0, new_yaw)

                            rotate_pose = deepcopy(current_pose)
                            rotate_pose.pose.orientation.x = q[0]
                            rotate_pose.pose.orientation.y = q[1]
                            rotate_pose.pose.orientation.z = q[2]
                            rotate_pose.pose.orientation.w = q[3]

                            navigator.goToPose(rotate_pose)
                            while not navigator.isTaskComplete():
                                rclpy.spin_once(sensor_monitor, timeout_sec=0.1)

                    print('>>> Interrupt routine complete. Resuming waypoints... <<<\n')

                # Resume remaining waypoints
                if interrupted_waypoint_idx is not None:
                    remaining = inspection_points[interrupted_waypoint_idx:]
                    if remaining:
                        navigator.followWaypoints(remaining)
                    else:
                        print('No remaining waypoints – navigation finished?')

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