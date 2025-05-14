#!/usr/bin/env python3

# TeraRanger Evo ROS2 Node
# Publishes raw data on /distance_raw and processed distance on /distance

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
import serial
import sys
import crcmod.predefined
from std_msgs.msg import Float64, String
from sensor_msgs.msg import Range

class TeraRangerEvoNode(Node):
    def __init__(self):
        super().__init__('teraranger_evo_node')
        
        # Declare parameters with descriptors
        name_descriptor = ParameterDescriptor(description='Device name for TeraRanger Evo sensor')
        port_descriptor = ParameterDescriptor(description='Serial port for TeraRanger Evo sensor')
        frame_descriptor = ParameterDescriptor(description='Frame name for TeraRanger Evo sensor')
        publish_descriptor = ParameterDescriptor(description='Publish rate in ms for TeraRanger Evo sensor')
        
        self.declare_parameter('device_name', 'TR_evo_3m', name_descriptor)
        self.declare_parameter('serial_port', '/dev/ttyLidar', port_descriptor)
        self.declare_parameter('frame_id', 'lidar_frame',frame_descriptor)
        self.declare_parameter('publish_rate_ms', 100.0, publish_descriptor)
        
        # Get parameters
        self.device_name = self.get_parameter('device_name').value
        self.port = self.get_parameter('serial_port').value
        self.frame = self.get_parameter('frame_id').value
        self.publish_rate_ms = self.get_parameter('publish_rate_ms').value
        
        # Publishers
        self.raw_publisher = self.create_publisher(String, self.device_name + '/distance_raw', 10)
        self.distance_publisher = self.create_publisher(Float64, self.device_name + '/distance', 10)
        self.range_publisher = self.create_publisher(Range, self.device_name + '/range', 10)
        
        # CRC function
        self.crc8_fn = crcmod.predefined.mkPredefinedCrcFun('crc-8')
        
        # Connect to sensor
        self.connect_to_sensor()
        
        # Timer for reading sensor
        self.timer = self.create_timer(self.publish_rate_ms/1000.0, self.timer_callback)  # 10Hz
        
        self.get_logger().info('TeraRanger Evo node initialized')
    
    def connect_to_sensor(self):
        try:
            self.get_logger().info(f'Attempting to open port {self.port}...')
            self.evo = serial.Serial(self.port, baudrate=115200, timeout=2)
            
            # Send the command "Binary mode"
            set_bin = bytes([0x00, 0x11, 0x02, 0x4C])
            
            # Flush input buffer
            self.evo.flushInput()
            
            # Write the binary command to the Evo
            self.evo.write(set_bin)
            
            # Flush output buffer
            self.evo.flushOutput()
            
            self.get_logger().info('Serial port opened successfully')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            sys.exit(1)
    
    def get_evo_range(self):
        try:
            # Read one byte
            data = self.evo.read(1)
            
            if data == b'T':
                # After T read 3 bytes
                frame = data + self.evo.read(3)
                
                if frame[3] == self.crc8_fn(frame[0:3]):
                    # Convert binary frame to decimal in shifting by 8 the frame
                    rng = frame[1] << 8
                    rng = rng | (frame[2] & 0xFF)
                    
                    # Publish raw value
                    raw_msg = String()
                    raw_msg.data = str(rng)
                    self.raw_publisher.publish(raw_msg)

                    # Flush input buffer
                    self.evo.flushInput()
                    
                    # Check special cases (limit values)
                    if rng == 65535:  # Sensor measuring above its maximum limit
                        return float('inf')
                    elif rng == 1:    # Sensor not able to measure
                        return float('nan')
                    elif rng == 0:    # Sensor detecting object below minimum range
                        return -float('inf')
                    else:
                        # Convert frame in meters
                        return float(rng / 1000.0)
                else:
                    self.get_logger().warning("CRC mismatch. Check connection.")
                    return None
            else:
                return None
                
        except serial.SerialException as e:
            self.get_logger().error(f"Device error: {e}")
            return None
    
    def timer_callback(self):
        distance = self.get_evo_range()
        
        if distance is not None:
            distance_msg = Float64()
            distance_msg.data = distance
            self.distance_publisher.publish(distance_msg)
            range_msg = Range()
            range_msg.header.stamp = self.get_clock().now().to_msg()
            range_msg.header.frame_id = self.frame 
            range_msg.radiation_type = Range.INFRARED # or ULTRASOUND
            range_msg.field_of_view = 0.03 # Radians 
            range_msg.min_range = 0.1 # Meters 
            range_msg.max_range = 3.0 # Meters 
            range_msg.range = float(distance)
            self.range_publisher.publish(range_msg)

    
    def destroy_node(self):
        if hasattr(self, 'evo') and self.evo.is_open:
            self.evo.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TeraRangerEvoNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
