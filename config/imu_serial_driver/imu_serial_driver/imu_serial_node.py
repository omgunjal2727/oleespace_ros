#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import serial
import math

class IMUSerialNode(Node):
    def __init__(self):
        super().__init__('imu_serial_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Create publisher
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        
        # Initialize serial connection
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1.0
            )
            self.get_logger().info(f'Connected to IMU on {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise
        
        # Create timer for reading serial data
        self.timer = self.create_timer(0.01, self.read_serial_data)  # 100Hz
        
        self.get_logger().info('IMU Serial Node started')
    
    def read_serial_data(self):
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                
                # Parse data: IMU,timestamp,ax,ay,az,gx,gy,gz
                if line.startswith('IMU,'):
                    parts = line.split(',')
                    if len(parts) == 8:
                        timestamp = int(parts[1])
                        ax = float(parts[2])
                        ay = float(parts[3])
                        az = float(parts[4])
                        gx = float(parts[5])
                        gy = float(parts[6])
                        gz = float(parts[7])
                        
                        # Create IMU message
                        imu_msg = Imu()
                        imu_msg.header = Header()
                        imu_msg.header.stamp = self.get_clock().now().to_msg()
                        imu_msg.header.frame_id = self.frame_id
                        
                        # Linear acceleration (m/s^2)
                        # ISM330DHCX outputs in g, convert to m/s^2
                        imu_msg.linear_acceleration.x = ax * 9.81
                        imu_msg.linear_acceleration.y = ay * 9.81
                        imu_msg.linear_acceleration.z = az * 9.81
                        
                        # Angular velocity (rad/s)
                        # ISM330DHCX outputs in dps, convert to rad/s
                        imu_msg.angular_velocity.x = math.radians(gx)
                        imu_msg.angular_velocity.y = math.radians(gy)
                        imu_msg.angular_velocity.z = math.radians(gz)
                        
                        # Orientation is not available (no magnetometer)
                        imu_msg.orientation_covariance[0] = -1.0
                        
                        # Set covariance matrices (tune these values based on your sensor)
                        # Linear acceleration covariance
                        imu_msg.linear_acceleration_covariance = [
                            0.01, 0.0, 0.0,
                            0.0, 0.01, 0.0,
                            0.0, 0.0, 0.01
                        ]
                        
                        # Angular velocity covariance
                        imu_msg.angular_velocity_covariance = [
                            0.001, 0.0, 0.0,
                            0.0, 0.001, 0.0,
                            0.0, 0.0, 0.001
                        ]
                        
                        # Publish message
                        self.imu_pub.publish(imu_msg)
                        
        except Exception as e:
            self.get_logger().error(f'Error reading serial data: {e}')
    
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