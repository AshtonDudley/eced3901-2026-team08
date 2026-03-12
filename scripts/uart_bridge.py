#!/usr/bin/env python3

# Author: Liam Legge
# Last Updated, Mar 12th 2026
# Python Translation

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import serial

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        # Create Publishers
        self.pub_distance0 = self.create_publisher(Int32, 'distance0', 10)
        self.pub_distance1 = self.create_publisher(Int32, 'distance1', 10)

        # subscribe to scan topic
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, rclpy.qos.qos_profile_sensor_data)

        # Start Outputting
        self.get_logger().info("=============================================")
        self.get_logger().info("UART Bridge Started")
        self.get_logger().info("=============================================")
        
        # Open connection to linux serial device
        try:
            self.serial_port = serial.Serial(
                port='/dev/ttyUSB4',
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0,          # Non-blocking read
                xonxoff=False,      # No software flow control
                rtscts=False,       # No hardware flow control
                dsrdtr=False
            )
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {e}")
            raise SystemExit

        self.buffer = ""

        # Create timer loop to read serial
        self.read_timer = self.create_timer(0.01, self.read_serial) # 10ms
        self.write_timer = self.create_timer(0.2, self.write_serial) # 200ms

        self.scan_data_min = -1.0

    def destroy_node(self):
        # Close the serial port on shutdown
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("Serial Port Closed")
        super().destroy_node()

    def read_serial(self):
        if not self.serial_port.is_open:
            return

        try:
            if self.serial_port.in_waiting > 0:
                # Read all available bytes and decode
                data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                self.buffer += data

                # Parse complete lines separated by '\n'
                while '\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\n', 1)
                    line = line.strip()

                    if not line:
                        continue

                    # Check for Dist1
                    if "Dist1" in line:
                        try:
                            val_str = line.split(":")[1]
                            msg = Int32()
                            msg.data = int(val_str)
                            self.pub_distance0.publish(msg)
                        except Exception:
                            self.get_logger().warn(f"Failed to parse line: {line}")

                    # Check for Dist2
                    elif "Dist2" in line:
                        try:
                            val_str = line.split(":")[1]
                            msg = Int32()
                            msg.data = int(val_str)
                            self.pub_distance1.publish(msg)
                        except Exception:
                            self.get_logger().warn(f"Failed to parse line: {line}")
                            
        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")

    def write_serial(self):
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            try:
                self.serial_port.write(data.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f"Error writing: {e}")
        else
            self.get_logger().error("Serial port not open for writing")

    def lidar_min_distance(self, scan_msg):
        if self.scan_data_min >= 0.0
            format_str = f"[TOPIC] LiDAR:{self.scan_data_min:.2f}\n"
            self.write_serial(format_str)

    def scan_callback(self, msg):
        min_dist = msg.range_max
        for range_val in msg.ranges:
            if msg.range_min < range_val < min_dist:
                min_dist = range_val
        self.scan_data_min = min_dist

def main(args=None):
    rclpy.init(args=args)
    try:
        node = SerialBridgeNode()
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()