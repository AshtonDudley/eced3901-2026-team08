#! /usr/bin/env python3

# Preemptive waypoint navigation with resume
# Interrupt current route → run new route → resume old route

from copy import deepcopy
import rclpy
import numpy as np

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


# -----------------------------------------------------------
# Quaternion helper (Euler → Quaternion)
# -----------------------------------------------------------
def get_quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]


# -----------------------------------------------------------
# Convert route [x, y, yaw] → PoseStamped waypoints
# -----------------------------------------------------------
def build_waypoints(navigator, route):
    points = []
    pose = PoseStamped()
    pose.header.frame_id = 'map'

    for pt in route:
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = pt[0]
        pose.pose.position.y = pt[1]

        q = get_quaternion_from_euler(0, 0, pt[2])
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        points.append(deepcopy(pose))

    return points


# -----------------------------------------------------------
# MAIN
# -----------------------------------------------------------
def main():
    rclpy.init()

    navigator = BasicNavigator()

    # -------------------------------------------------------
    # DEFINE ROUTES  (Edit these for your robot)
    # -------------------------------------------------------

    # Route A — Example: go deliver cargo
    route_cargo = [
        [2.0, 0.6, 0.0],
        [3.5, 0.0, 3.14]
    ]

    # Route B — Example: go home
    route_home = [
        [2.0, 0.6, 3.14],
        [0.0, -0.5, 0.0]
    ]

    # -------------------------------------------------------
    # MISSION STATE VARIABLES
    # -------------------------------------------------------
    current_route = route_cargo
    current_index = 0
    route_stack = []
    interrupt_triggered = False

    # -------------------------------------------------------
    # WAIT FOR NAV2
    # -------------------------------------------------------
    navigator.waitUntilNav2Active()
    print("Nav2 is active.")

    # -------------------------------------------------------
    # START FIRST ROUTE
    # -------------------------------------------------------
    print("Starting Cargo Route")
    navigator.followWaypoints(build_waypoints(navigator, current_route))

    # -------------------------------------------------------
    # MAIN LOOP
    # -------------------------------------------------------
    i = 0

    while rclpy.ok():

        # -----------------------------------------------
        # INTERRUPT CONDITION (CHANGE THIS!)
        # Demo: interrupt after reaching waypoint 1
        # -----------------------------------------------
        if (not interrupt_triggered) and current_index >= 2:
            print("\nINTERRUPT: Switching to HOME route\n")

            interrupt_triggered = True

            # Cancel current navigation task
            navigator.cancelTask()

            # Save remaining part of current route
            remaining = current_route[current_index:]
            route_stack.append(remaining)

            # Start new route
            current_route = route_home
            current_index = 0

            navigator.followWaypoints(
                build_waypoints(navigator, current_route)
            )

            continue

        # -----------------------------------------------
        # IF NAVIGATION STILL RUNNING
        # -----------------------------------------------
        if not navigator.isTaskComplete():

            feedback = navigator.getFeedback()
            if feedback:
                current_index = feedback.current_waypoint
                i += 1

                if i % 5 == 0:
                    print(
                        f"Executing waypoint "
                        f"{current_index + 1}/{len(current_route)}"
                    )

        # -----------------------------------------------
        # TASK FINISHED
        # -----------------------------------------------
        else:
            result = navigator.getResult()

            if result == TaskResult.SUCCEEDED:
                print("\nRoute completed")

                # Resume previous route if saved
                if route_stack:
                    print("Resuming previous route...\n")

                    current_route = route_stack.pop()
                    current_index = 0

                    navigator.followWaypoints(
                        build_waypoints(navigator, current_route)
                    )
                    continue

                else:
                    print("🏁 Mission complete")
                    break

            elif result == TaskResult.CANCELED:
                print("Route canceled")
                break

            elif result == TaskResult.FAILED:
                print("Route failed")
                break

    exit(0)


if __name__ == '__main__':
    main()