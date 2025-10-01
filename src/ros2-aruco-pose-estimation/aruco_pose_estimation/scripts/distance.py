#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from aruco_interfaces.msg import ArucoMarkers
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Point
import math
import time
import json

class ArucoDistanceMonitor(Node):
    def __init__(self):
        super().__init__('aruco_distance_monitor')
        
        # Subscribe to ArUco markers
        self.subscription = self.create_subscription(
            ArucoMarkers,
            '/aruco/markers',
            self.marker_callback,
            10
        )
        
        # Publishers for distance data
        self.distance_publisher = self.create_publisher(
            Float32MultiArray,
            '/aruco/distances',
            10
        )
        
        # Publisher for detailed info as JSON string
        self.info_publisher = self.create_publisher(
            String,
            '/aruco/distance_info',
            10
        )
        
        # Publishers for specific marker positions (your 4 target markers)
        self.marker102_publisher = self.create_publisher(
            Point,
            '/aruco/marker_102/position',
            10
        )
        
        self.marker101_publisher = self.create_publisher(
            Point,
            '/aruco/marker_101/position',
            10
        )
        
        self.marker122_publisher = self.create_publisher(
            Point,
            '/aruco/marker_122/position',
            10
        )
        
        self.marker121_publisher = self.create_publisher(
            Point,
            '/aruco/marker_121/position',
            10
        )
        
        self.get_logger().info('ArUco Distance Monitor started')
        self.get_logger().info('Monitoring all detected markers')
        self.get_logger().info('Publishing distance data to topics')
        
    def calculate_distance_cm(self, position):
        """Calculate euclidean distance using sqrt formula in centimeters"""
        x = position.x * 100  # Convert to cm
        y = position.y * 100  # Convert to cm
        z = position.z * 100  # Convert to cm
        
        # Distance = sqrt(x² + y² + z²)
        distance = math.sqrt(x*x + y*y + z*z)
        
        return distance, x, y, z
    
    def marker_callback(self, msg):
        """Process all detected markers"""
        if not msg.marker_ids:
            return
            
        current_time = time.time()
        
        self.get_logger().info(f"=== DETECTED {len(msg.marker_ids)} MARKERS ===")
        
        # Prepare data for publishing
        distances_array = Float32MultiArray()
        distance_info_list = []
        
        for i, marker_id in enumerate(msg.marker_ids):
            if i < len(msg.poses):
                position = msg.poses[i].position
                
                # Calculate distance and coordinates in cm
                distance_cm, x_cm, y_cm, z_cm = self.calculate_distance_cm(position)
                
                # Add to distances array: [marker_id, distance, x, y, z]
                distances_array.data.extend([
                    float(marker_id),
                    distance_cm,
                    x_cm,
                    y_cm,
                    z_cm
                ])
                
                # Create detailed info
                info = {
                    'marker_id': int(marker_id),
                    'timestamp': current_time,
                    'distance_cm': round(distance_cm, 1),
                    'position_cm': {
                        'x': round(x_cm, 1),
                        'y': round(y_cm, 1),
                        'z': round(z_cm, 1)
                    }
                }
                distance_info_list.append(info)
                
                # Publish specific marker positions for your 4 target IDs
                if marker_id == 102:
                    marker102_pos = Point()
                    marker102_pos.x = x_cm
                    marker102_pos.y = y_cm
                    marker102_pos.z = z_cm
                    self.marker102_publisher.publish(marker102_pos)
                    
                elif marker_id == 101:
                    marker101_pos = Point()
                    marker101_pos.x = x_cm
                    marker101_pos.y = y_cm
                    marker101_pos.z = z_cm
                    self.marker101_publisher.publish(marker101_pos)
                    
                elif marker_id == 122:
                    marker122_pos = Point()
                    marker122_pos.x = x_cm
                    marker122_pos.y = y_cm
                    marker122_pos.z = z_cm
                    self.marker122_publisher.publish(marker122_pos)
                    
                elif marker_id == 121:
                    marker121_pos = Point()
                    marker121_pos.x = x_cm
                    marker121_pos.y = y_cm
                    marker121_pos.z = z_cm
                    self.marker121_publisher.publish(marker121_pos)
                
                # Log marker information
                self.get_logger().info(
                    f"Marker {marker_id}: "
                    f"Distance = {distance_cm:.1f}cm, "
                    f"Position = ({x_cm:.1f}, {y_cm:.1f}, {z_cm:.1f})cm"
                )
        
        # Publish distance array
        self.distance_publisher.publish(distances_array)
        
        # Publish detailed info as JSON
        info_msg = String()
        info_msg.data = json.dumps(distance_info_list, indent=2)
        self.info_publisher.publish(info_msg)
        
        self.get_logger().info("=" * 50)

def main(args=None):
    rclpy.init(args=args)
    monitor = ArucoDistanceMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\nShutting down ArUco Distance Monitor...")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()