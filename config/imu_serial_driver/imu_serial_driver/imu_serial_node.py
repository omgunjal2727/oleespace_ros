#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import serial
import math
import numpy as np

class IMUSerialNode(Node):
    def __init__(self):
        super().__init__('imu_serial_node')
        
        # --- CONFIGURATION ---
        self.declare_parameter('serial_port', '/dev/ttyACM0') 
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)  # Changed from data_raw
        
        # --- CALIBRATION VARIABLES ---
        self.is_calibrated = False
        self.calibration_samples = 0
        self.max_samples = 100  # Take 100 readings (~2 seconds) to average
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_z = 0.0
        
        # Storage for calibration data
        self.calib_x_list = []
        self.calib_y_list = []
        
        # Connect Serial
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.1
            )
            self.get_logger().info(f'Connected to IMU on {self.serial_port}')
            self.get_logger().info('--- CALIBRATING: KEEP ROBOT STILL FOR 3 SECONDS ---')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            return
        
        self.timer = self.create_timer(0.02, self.read_serial_data)
    
    def read_serial_data(self):
        if not hasattr(self, 'ser') or not self.ser.is_open:
            return

        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                parts = line.split(',')
                
                # Check if we have orientation data (10 values) or just IMU (6 values)
                if len(parts) >= 10:
                    # NEW FORMAT: ax,ay,az,gx,gy,gz,qx,qy,qz,qw
                    try:
                        # 1. GET RAW VALUES
                        raw_ax = float(parts[0])
                        raw_ay = float(parts[1])
                        raw_az = float(parts[2])
                        
                        raw_gx = float(parts[3])
                        raw_gy = float(parts[4])
                        raw_gz = float(parts[5])
                        
                        # Quaternion orientation
                        qx = float(parts[6])
                        qy = float(parts[7])
                        qz = float(parts[8])
                        qw = float(parts[9])

                        # 2. CALIBRATION PHASE
                        if not self.is_calibrated:
                            if self.calibration_samples < self.max_samples:
                                self.calib_x_list.append(raw_ax)
                                self.calib_y_list.append(raw_ay)
                                self.calibration_samples += 1
                                return # Don't publish yet
                            else:
                                # Calculate Average Error
                                self.offset_x = sum(self.calib_x_list) / len(self.calib_x_list)
                                self.offset_y = sum(self.calib_y_list) / len(self.calib_y_list)
                                
                                self.get_logger().info(f'Calibration Done! Offsets -> X: {self.offset_x:.2f}, Y: {self.offset_y:.2f}')
                                self.is_calibrated = True

                        # 3. PUBLISH CORRECTED DATA
                        imu_msg = Imu()
                        imu_msg.header = Header()
                        imu_msg.header.stamp = self.get_clock().now().to_msg()
                        imu_msg.header.frame_id = self.frame_id
                        
                        # Apply Offsets (accel already in m/s^2 from Arduino)
                        imu_msg.linear_acceleration.x = raw_ax - self.offset_x
                        imu_msg.linear_acceleration.y = raw_ay - self.offset_y
                        imu_msg.linear_acceleration.z = raw_az

                        # GYRO (already in rad/s from Arduino)
                        imu_msg.angular_velocity.x = raw_gx
                        imu_msg.angular_velocity.y = raw_gy
                        imu_msg.angular_velocity.z = raw_gz
                        
                        # ORIENTATION (quaternion)
                        imu_msg.orientation.x = qx
                        imu_msg.orientation.y = qy
                        imu_msg.orientation.z = qz
                        imu_msg.orientation.w = qw
                        
                        # Set covariance (use -1 if unknown, or set proper values)
                        imu_msg.orientation_covariance[0] = 0.01  # Small value = we have orientation
                        imu_msg.angular_velocity_covariance[0] = 0.01
                        imu_msg.linear_acceleration_covariance[0] = 0.01
                        
                        self.imu_pub.publish(imu_msg)

                    except ValueError as e:
                        self.get_logger().warn(f'Parse error: {e}')
                        
                elif len(parts) >= 6:
                    # OLD FORMAT: Just ax,ay,az,gx,gy,gz (no orientation)
                    self.get_logger().warn_once('Receiving old format without orientation')
                    # Handle old format as before...
                    
        except Exception as e:
            self.get_logger().warn(f'Serial read error: {e}')
    
    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = IMUSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()