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

from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import serial
import time

"""
Basic stock inspection demo. In this demonstration, the expectation
is that there are cameras or RFID sensors mounted on the robots
collecting information about stock quantity and location.
"""

# This fxn converts Euler angles to a quaternion.
# Author: AutomaticAddison.com
import numpy as np # Scientific computing library for Python
 
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
    rclpy.init()

    navigator = BasicNavigator()

    # Inspection route, probably read in from a file for a real application
    # from either a map or drive and repeat.
    # [ X-pos, Y-pos, Theta-yaw ]
    phase1_route = [
        #Necessary half way poses
        [2.0, -0.6, 0.0],
        [2.6, 0.1, 0.0],
        #Drop off cargo
        [3.85, -0.6,3.14],
        #Reset
        [2.8, -0.45, 3.14],
        #Pick up cargo
        [3.45, 0.0, 3.14]]
    
    phase2_route = [
        #Necessary half way pose
        [2.2, -0.65, 3.14],
        #Head back to origin
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

    # Create cmd_vel publisher for dead reckoning
    cmd_vel_pub = navigator.create_publisher(Twist, 'cmd_vel', 10)

    # Helper to build waypoint list from a route
    def build_waypoints(route):
        points = []
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        for pt in route:
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            q = get_quaternion_from_euler(0, 0, pt[2])
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            points.append(deepcopy(pose))
        return points

    # Open UART
    try:
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    except serial.SerialException as e:
        print(f'Serial open failed: {e}')
        ser = None

    # ---- PHASE 1: Navigate to pick up cargo ----
    phase1_points = build_waypoints(phase1_route)
    navigator.followWaypoints(phase1_points)

    if ser:
        ser.write(b'\xf1')  # Magnet ON at start
        print('Magnet ON')

    i = 0
    last_waypoint = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback:
            if feedback.current_waypoint > last_waypoint:
                reached_waypoint = feedback.current_waypoint + 1
                if reached_waypoint == 4:
                    if ser:
                        ser.write(b'\xf2')  # Magnet OFF at drop off
                        print('Magnet OFF (drop off)')
                elif reached_waypoint == 5:
                    if ser:
                        ser.write(b'\xf1')  # Magnet ON at pick up
                        print('Magnet ON (pick up)')
                last_waypoint = feedback.current_waypoint
            if i % 5 == 0:
                print('Phase 1 waypoint: ' +
                    str(feedback.current_waypoint + 1) + '/' + str(len(phase1_points)))

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Phase 1 complete. Starting dead reckoning...')
    else:
        print(f'Phase 1 ended with result: {result}')

    # ---- DEAD RECKONING: back up 5 cm then rotate left/right ----
    twist = Twist()

    # Back up 5 cm (0.05 m at 0.1 m/s = 0.5 seconds)
    twist.linear.x = -0.1
    print('Backing up 5 cm...')
    start_time = time.time()
    while time.time() - start_time < 0.5: #makes it back up 5cm 
        cmd_vel_pub.publish(twist)
        time.sleep(0.05)
    cmd_vel_pub.publish(Twist())  # Stop before rotating

    # Rotate left for 2.5 seconds
    twist = Twist()
    twist.angular.z = 0.5  # rad/s counterclockwise
    print('Rotating left...')
    start_time = time.time()
    while time.time() - start_time < 2.5:
        cmd_vel_pub.publish(twist)
        time.sleep(0.05)

    # Rotate right for 2.5 seconds
    twist.angular.z = -0.5  # rad/s clockwise
    print('Rotating right...')
    start_time = time.time()
    while time.time() - start_time < 3.0:
        cmd_vel_pub.publish(twist)
        time.sleep(0.05)

    twist.angular.z = 0.5  # rad/s counterclockwise
    print('Rotating left...')
    start_time = time.time()
    while time.time() - start_time < 3.5:
        cmd_vel_pub.publish(twist)
        time.sleep(0.05)

    # Stop the robot
    cmd_vel_pub.publish(Twist())
    print('Dead reckoning complete.')

    # ---- PHASE 2: Resume nav stack to return home ----
    phase2_points = build_waypoints(phase2_route)
    navigator.followWaypoints(phase2_points)

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Phase 2 waypoint: ' +
                str(feedback.current_waypoint + 1) + '/' + str(len(phase2_points)))

    if ser:
        ser.write(b'\xf2')  # Magnet OFF at end
        print('Magnet OFF (end)')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Mission complete!')
    elif result == TaskResult.CANCELED:
        print('Phase 2 was canceled.')
    elif result == TaskResult.FAILED:
        print('Phase 2 failed!')

    # go back to start
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # navigator.goToPose(initial_pose)
    # while not navigator.isTaskComplete():
    #     pass

    exit(0)


if __name__ == '__main__':
    main()
