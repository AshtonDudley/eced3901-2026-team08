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
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

# Create Interrupt Node
class SensorMonitor(node):
    def __init__(self):
        self.node = node
        self.lidar_front = None
        self.ultrasonic_front = None
        self.lidar_angle_idx = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriptions
        self.lidar_sub = node.create_subscription(LaserScan, '/scan', self.lidar_callback, sensor_qos)
        self.ultrasonic_sub = node.create_subscription(LaserScan, '/ultrasonic_scan', self.ultrasonic_callback, sensor_qos)

    def lidar_callback(self, msg):
        if self.lidar_angle_idx is None:
            idx = int((0.0 - msg.angle_min) / msg.angle_increment)
            if 0 <= idx < len(msg.ranges):
                self.lidar_angle_idx = idx
            else:
                self.node.get_logger().warn("Lidar angle index out of range")
                return
        if self.lidar_angle_idx is not None:
            self.lidar_front = msg.ranges[self.lidar_angle_idx]

    def ultrasonic_callback(self, msg):
        if msg.ranges:
            self.ultrasonic_front = msg.ranges[0]

    def difference(self, threshold):
        if self.lidar_front is None or self.ultrasonic_front is None:
            return False
        if self.lidar_front - self.ultrasonic_front < 0:
            return False
        return self.lidar_front - self.ultrasonic_front > threshold

"""
Basic stock inspection demo. In this demonstration, the expectation
is that there are cameras or RFID sensors mounted on the robots
collecting information about stock quantity and location.
"""

# This fxn converts Euler angles to a quaternion.
# Author: AutomaticAddison.com
 
def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]
  

def main():

    # Define Params
    DIFF_THRESHOLD = 0.15
    TARGET_DISTANCE = 0.01
    STEP_SIZE = 0.05
    COOLDOWN_TIME = 1.0
    last_interrupt_time = None
    interrupted_waypoint_idx = None

    rclpy.init()

    navigator = BasicNavigator()

    # Inspection route, probably read in from a file for a real application
    # from either a map or drive and repeat.
    # [ X-pos, Y-pos, Theta-yaw ]
    inspection_route = [
        #Necessary half way poses
        [2.0, -0.6, 0.0],
        [2.6, 0.1, 0.0],
        #Drop off cargo
        [3.30, 0.0, 3.14],
        [2.2, -0.65, 3.14],
        [-0.5, 0.0, 0.0]]

    # Set our demo's initial pose
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = 0.0
    # initial_pose.pose.position.y = 0.0
    # initial_pose.pose.orientation.z = 0.0
    # initial_pose.pose.orientation.w = 0.0
    # navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    # Send our route
    inspection_points = []
    inspection_pose = PoseStamped()
    inspection_pose.header.frame_id = 'map'
    inspection_pose.header.stamp = navigator.get_clock().now().to_msg()
    #inspection_pose.pose.orientation.z = 0.0
    #inspection_pose.pose.orientation.w = 1.0
    for pt in inspection_route:    
        inspection_pose.pose.position.x = pt[0]
        inspection_pose.pose.position.y = pt[1]
        q = get_quaternion_from_euler(0,0,pt[2])
        inspection_pose.pose.orientation.x = q[0]
        inspection_pose.pose.orientation.y = q[1]      
        inspection_pose.pose.orientation.z = q[2]
        inspection_pose.pose.orientation.w = q[3]  
        inspection_points.append(deepcopy(inspection_pose))
    navigator.followWaypoints(inspection_points)

    # Do something during our route (e.x. AI to analyze stock information or upload to the cloud)
    # Simply the current waypoint ID for the demonstation
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(inspection_points)))

        # EDIT THIS, THIS IS FROM DEEPSEEK AND NEEDS TO BE ADJUSTED
        # Check for interrupt condition
        now = navigator.get_clock().now().seconds_nanoseconds()[0]
        if last_interrupt_time is None or (now - last_interrupt_time) > COOLDOWN_SEC:
            if sensor_monitor.difference_exceeds(DIFF_THRESHOLD):
                print('\n>>> Interrupt triggered! Difference large. <<<\n')
                last_interrupt_time = now
                if feedback:
                    interrupted_waypoint_idx = feedback.current_waypoint  # 0‑based
                else:
                    interrupted_waypoint_idx = 0

                # Cancel current navigation task
                navigator.cancelTask()
                while not navigator.isTaskComplete():
                    rclpy.spin_once(sensor_node, timeout_sec=0.1)
                result = navigator.getResult()
                print(f'Cancelled with result: {result}')

                # ----------------------------------------------------
                #  Interrupt routine: move toward object and rotate
                # ----------------------------------------------------
                current_pose = navigator.getCurrentPose()
                if current_pose is None:
                    print('Failed to get current pose – skipping interrupt routine')
                else:
                    # Move forward step by step until close to object
                    steps = 0
                    max_steps = 10
                    while (sensor_monitor.ultrasonic_front is not None and sensor_monitor.ultrasonic_front > TARGET_DIST and steps < max_steps):
                        # Compute next pose forward by STEP_SIZE in current direction
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

                        # Wait for this step, but allow early cancel if ultrasonic already low
                        while not navigator.isTaskComplete():
                            rclpy.spin_once(sensor_node, timeout_sec=0.1)
                            if (sensor_monitor.ultrasonic_front is not None and
                                    sensor_monitor.ultrasonic_front <= TARGET_DIST):
                                navigator.cancelTask()
                                break

                        current_pose = navigator.getCurrentPose()
                        steps += 1

                    # Now rotate 360° in place
                    if current_pose is not None:
                        yaw = get_yaw_from_quaternion(current_pose.pose.orientation)
                        new_yaw = yaw + 2.0 * math.pi
                        q = get_quaternion_from_euler(0, 0, new_yaw)

                        rotate_pose = deepcopy(current_pose)
                        rotate_pose.pose.orientation.x = q[0]
                        rotate_pose.pose.orientation.y = q[1]
                        rotate_pose.pose.orientation.z = q[2]
                        rotate_pose.pose.orientation.w = q[3]

                        navigator.goToPose(rotate_pose)
                        while not navigator.isTaskComplete():
                            rclpy.spin_once(sensor_node, timeout_sec=0.1)

                    print('>>> Interrupt routine complete. Resuming waypoints... <<<\n')

                # Resume remaining waypoints
                if interrupted_waypoint_idx is not None:
                    remaining = inspection_points[interrupted_waypoint_idx:]
                    if remaining:
                        navigator.followWaypoints(remaining)
                    else:
                        print('No remaining waypoints – navigation finished?')
                # Continue the outer loop (FROM DEEPSEEK)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Inspection complete! Returning to start...')
    elif result == TaskResult.CANCELED:
        print('Inspection was canceled. Returning to start...')
    elif result == TaskResult.FAILED:
        print('Inspection failed! Returning to start...')

    # go back to start
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # navigator.goToPose(initial_pose)
    # while not navigator.isTaskComplete():
    #     pass

    exit(0)


if __name__ == '__main__':
    main()
