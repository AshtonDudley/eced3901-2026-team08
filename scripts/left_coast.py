#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
# Updated March 12, 2026 - By: Megan Neville
# Navigation 2 Waypoint Follower - Ultrasonic-Only Lifeboat Rescue

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
import serial

# ==========================================
# HELPER FUNCTIONS
# ==========================================

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

def normalize_angle(angle):
    # Keeps angles perfectly bounded between -PI and PI
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

# ==========================================
# SENSOR MONITOR NODE (Ultrasonic Only)
# ==========================================

class SensorMonitor(Node):
    def __init__(self):
        super().__init__('sensor_monitor')
        self.ultrasonic_front = None
        self.amcl_pose = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.ultrasonic_sub = self.create_subscription(LaserScan, '/ultrasonic_scan', self.ultrasonic_callback, sensor_qos)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

        # Publisher for Manual Motor Override
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def pose_callback(self, msg):
        self.amcl_pose = msg.pose.pose

    def ultrasonic_callback(self, msg):
        if msg.ranges:
            val = msg.ranges[0]
            if not math.isinf(val) and not math.isnan(val):
                self.ultrasonic_front = val

# ==========================================
# MAIN NAVIGATION LOGIC
# ==========================================

def main():
    rclpy.init()

    # Interrupt Parameters
    DETECT_DISTANCE = 0.20  # Absolute distance trigger (< 20cm)
    TARGET_DISTANCE = 0.05  # Stop driving when 5cm away
    MANUAL_SPEED = 0.05     # Drive at 5 cm/s
    SPIN_SPEED = 0.8        # Positive = Counter-Clockwise spin (rad/s)
    COOLDOWN_TIME = 15.0    # Prevent back-to-back triggers
    
    last_interrupt_time = None
    sensor_monitor = SensorMonitor()
    navigator = BasicNavigator()

    # [ X-pos, Y-pos, Theta-yaw ]
    inspection_route = [
        [1.8, 0.6, 0.78],   # 1. Necessary half way poses
        [2.6, 0.1, 0.0],    # 2. Necessary half way poses
        [3.85, 0.6, 1.9],   # 3. Drop off cargo
        [2.8, 0.3, 3.14],   # 4. Reset
        [3.45, -0.2, 3.20], # 5. Pick up cargo
        [1.8, 0.6, 3.14],   # 6. Necessary half way pose
        [-0.5, -0.5, 0.0]   # 7. Head back to origin
    ]

    navigator.waitUntilNav2Active()

    # Construct the Pose messages
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

    # Open UART for Servo/Mechanism
    try:
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    except serial.SerialException as e:
        print(f"Failed to open UART: {e}")
        ser = None

    # Global tracking variables to handle route slicing
    global_wp_offset = 0
    last_global_wp = 0
    i = 0

    if ser:
        ser.write(b'\xf3')

    # Main Navigation Loop
    while not navigator.isTaskComplete():
        i += 1
        rclpy.spin_once(sensor_monitor, timeout_sec=0.1)
        feedback = navigator.getFeedback()

        if feedback:
            # Calculate the true global waypoint index regardless of interruptions
            current_global_wp = global_wp_offset + feedback.current_waypoint
            
            # Feedback loop for dropping cargo
            if current_global_wp > last_global_wp:
                completed_waypoint = last_global_wp + 1
                
                # Trigger UART at specific global waypoints
                if completed_waypoint == 3 or completed_waypoint == 5:
                    if ser:
                        print(f"--> Triggering UART at Waypoint {completed_waypoint}")
                        ser.write(b'\xf3')
                        
                last_global_wp = current_global_wp 

            if i % 10 == 0: 
                print(f'Executing waypoint: {current_global_wp + 1}/{len(inspection_points)}')

        # ==========================================
        # INTERRUPT: DETECT, OVERRIDE, ALIGN, PICK UP
        # ==========================================
        now = time.time()
        if last_interrupt_time is None or (now - last_interrupt_time) > COOLDOWN_TIME:
            
            # PURE ULTRASONIC DISTANCE CHECK
            if sensor_monitor.ultrasonic_front is not None and sensor_monitor.ultrasonic_front < DETECT_DISTANCE:
                # Capture the exact distance of the lifeboat to use for the sweep threshold
                boat_distance = sensor_monitor.ultrasonic_front
                print(f'\n>>> LIFEBOAT DETECTED at {boat_distance:.2f}m! Halting Nav2... <<<')
                last_interrupt_time = time.time()
                
                # Save where we were in the list
                if feedback:
                    current_local_wp = feedback.current_waypoint
                else:
                    current_local_wp = 0

                # ---------------------------------------------------------
                # 1. CANCEL NAV2
                # ---------------------------------------------------------
                navigator.cancelTask()
                while not navigator.isTaskComplete():
                    rclpy.spin_once(sensor_monitor, timeout_sec=0.1)

                # ---------------------------------------------------------
                # 2. ALIGNMENT SWEEP: FIND THE CENTER OF THE LIFEBOAT
                # ---------------------------------------------------------
                if sensor_monitor.amcl_pose:
                    print('>>> Commencing Dynamic Sweep to find Lifeboat Center... <<<')
                    twist_msg = Twist()
                    
                    def get_current_yaw():
                        rclpy.spin_once(sensor_monitor, timeout_sec=0.05)
                        return get_yaw_from_quaternion(sensor_monitor.amcl_pose.orientation)

                    # Dynamic Thresholds based on actual boat distance
                    # Assume we hit an edge if the distance jumps by 10cm
                    EDGE_DROP_OFF = 0.10 
                    clear_distance = boat_distance + EDGE_DROP_OFF

                    # --- Phase A: Rotate Right until we "fall off" the right edge ---
                    twist_msg.angular.z = -0.3 
                    sweep_start = time.time()
                    while (sensor_monitor.ultrasonic_front is not None and 
                           sensor_monitor.ultrasonic_front < clear_distance and 
                           (time.time() - sweep_start) < 5.0):
                        sensor_monitor.cmd_vel_pub.publish(twist_msg)
                        rclpy.spin_once(sensor_monitor, timeout_sec=0.05)
                        
                    # Record Right Edge
                    twist_msg.angular.z = 0.0
                    sensor_monitor.cmd_vel_pub.publish(twist_msg)
                    time.sleep(0.5) 
                    yaw_right = get_current_yaw()

                    # --- Phase B: Rotate Left until we "fall off" the left edge ---
                    twist_msg.angular.z = 0.3 
                    sweep_start = time.time()
                    
                    # Wait until we see the lifeboat again (distance drops back down)
                    while (sensor_monitor.ultrasonic_front is None or 
                           sensor_monitor.ultrasonic_front > (boat_distance + 0.05)) and (time.time() - sweep_start) < 3.0:
                        sensor_monitor.cmd_vel_pub.publish(twist_msg)
                        rclpy.spin_once(sensor_monitor, timeout_sec=0.05)
                        
                    # Now keep spinning left until we lose it again on the other side
                    while (sensor_monitor.ultrasonic_front is not None and 
                           sensor_monitor.ultrasonic_front < clear_distance and 
                           (time.time() - sweep_start) < 8.0):
                        sensor_monitor.cmd_vel_pub.publish(twist_msg)
                        rclpy.spin_once(sensor_monitor, timeout_sec=0.05)
                        
                    # Record Left Edge
                    twist_msg.angular.z = 0.0
                    sensor_monitor.cmd_vel_pub.publish(twist_msg)
                    time.sleep(0.5)
                    yaw_left = get_current_yaw()

                    # --- Phase C: Calculate Midpoint and Align ---
                    angular_diff = normalize_angle(yaw_left - yaw_right)
                    target_yaw = normalize_angle(yaw_right + (angular_diff / 2.0))
                    print(f"Calculated Center: {target_yaw:.2f}. Aligning...")

                    twist_msg.angular.z = -0.2 # Spin slowly back to the right
                    align_start = time.time()
                    while (time.time() - align_start) < 5.0:
                        current_yaw = get_current_yaw()
                        error = normalize_angle(target_yaw - current_yaw)
                        # Break when error is tiny, or if we overshoot it
                        if abs(error) < 0.05 or error > 0:
                            break
                        sensor_monitor.cmd_vel_pub.publish(twist_msg)

                    twist_msg.angular.z = 0.0
                    sensor_monitor.cmd_vel_pub.publish(twist_msg)
                    print('>>> Alignment complete! <<<')
                    time.sleep(0.5) 

                # ---------------------------------------------------------
                # 3. OVERRIDE: INCH FORWARD TO 5cm
                # ---------------------------------------------------------
                print('>>> Driving forward to 5cm... <<<')
                twist_msg = Twist()
                twist_msg.linear.x = MANUAL_SPEED 
                
                start_drive = time.time()
                while (sensor_monitor.ultrasonic_front is not None and 
                       sensor_monitor.ultrasonic_front > TARGET_DISTANCE and
                       (time.time() - start_drive) < 10.0):
                    sensor_monitor.cmd_vel_pub.publish(twist_msg)
                    rclpy.spin_once(sensor_monitor, timeout_sec=0.05)
                    
                twist_msg.linear.x = 0.0
                sensor_monitor.cmd_vel_pub.publish(twist_msg)

                # ---------------------------------------------------------
                # 4. OVERRIDE: COUNTER-CLOCKWISE SPIN
                # ---------------------------------------------------------
                print('>>> Commencing CCW Rescue Spin... <<<')
                twist_msg.angular.z = SPIN_SPEED 
                
                start_rotate = time.time()
                rotate_duration = (2.0 * math.pi) / twist_msg.angular.z 
                
                while (time.time() - start_rotate) < rotate_duration:
                    sensor_monitor.cmd_vel_pub.publish(twist_msg)
                    rclpy.spin_once(sensor_monitor, timeout_sec=0.05)
                    
                twist_msg.angular.z = 0.0
                sensor_monitor.cmd_vel_pub.publish(twist_msg)

                print('>>> Rescue Complete. Resuming Nav2 Route... <<<\n')

                # ---------------------------------------------------------
                # 5. RESUME: HAND CONTROL BACK TO NAV2
                # ---------------------------------------------------------
                global_wp_offset += current_local_wp
                last_global_wp = global_wp_offset 
                
                remaining = inspection_points[global_wp_offset:]
                if remaining:
                    navigator.followWaypoints(remaining)


    # Final Cleanup
    if ser:
        ser.write(b'\xf3')

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