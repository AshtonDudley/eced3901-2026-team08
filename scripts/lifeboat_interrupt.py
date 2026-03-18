# Program not to be used, just for reference

from copy import deepcopy
import math
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import LaserScan

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

def main():

    # Define Params
    DIFF_THRESHOLD = 0.2
    TARGET_DISTANCE = 0.01
    STEP_SIZE = 0.05
    COOLDOWN_TIME = 2.0
    last_interrupt_time = None
    interrupted_waypoint_idx = None

    rclpy.init()

    # Define node for sensor monitoring
    sensor_node = rclpy.create_node('sensor_monitor')
    sensor_monitor = SensorMonitor(sensor_node)

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        # Process sensor callbacks
        rclpy.spin_once(sensor_node, timeout_sec=0.01)

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
                    while (sensor_monitor.ultrasonic_front is not None and
                           sensor_monitor.ultrasonic_front > TARGET_DIST and
                           steps < max_steps):
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
                # Continue the outer loop (now monitoring the resumed task)