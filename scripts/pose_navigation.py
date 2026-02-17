#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from copy import deepcopy
import numpy as np
import time


# Convert Euler → Quaternion
def get_quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

def gated_check(idx): 
    print(f"Running gated check for waypoint {idx}...") 
    time.sleep(10)
    return True

# Subscriber node that waits for a "continue" signal
class ContinueGate(Node):
    def __init__(self):
        super().__init__('continue_gate')
        self.allow_continue = False
        self.subscription = self.create_subscription(
            Bool,
            '/continue_signal',
            self.callback,
            10
        )

    def callback(self, msg):
        if msg.data:
            self.get_logger().info("Continue signal received.")
            self.allow_continue = True


def main():
    rclpy.init()

    navigator = BasicNavigator()
    gate = ContinueGate()

    navigator.waitUntilNav2Active()

    # Waypoints: [x, y, yaw]
    inspection_route = [
        [1.33, 0.46, 0.78],
        [3.55, 0.0, 1.57],
        [1.78, 0.46, 0.78],
        [-0.2, -0.2, 0.0]
    ]

    # Choose which waypoint numbers require a "true" message
    # Wait only after waypoint 2
    gated_waypoints = [2]

    # Convert to PoseStamped list
    inspection_points = []
    for pt in inspection_route:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = pt[0]
        pose.pose.position.y = pt[1]
        q = get_quaternion_from_euler(0, 0, pt[2])
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        inspection_points.append(deepcopy(pose))

    # Main loop: go to each waypoint
    for idx, pose in enumerate(inspection_points, start=1):
        print(f"\n=== Sending waypoint {idx}/{len(inspection_points)} ===")
        navigator.goToPose(pose)

        # Wait until robot reaches the waypoint
        while not navigator.isTaskComplete():
            rclpy.spin_once(gate, timeout_sec=0.1)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"Reached waypoint {idx}.")
        else:
            print("Navigation failed or canceled.")
            break

        # Decide whether to wait or auto-continue
        if idx in gated_waypoints:
            print(f"Waypoint {idx} is gated. Running check") 
            while not gated_check(idx): 
                rclpy.spin_once(navigator, timeout_sec=0.1) 
            print("Gated check passed. Continuing")
        else:
            print("Auto‑continuing to next waypoint")

    print("\nAll waypoints completed.")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
