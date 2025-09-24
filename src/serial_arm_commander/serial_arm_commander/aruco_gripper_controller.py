#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import UInt8
from std_srvs.srv import SetBool
import math
import time

class ArucoGripperController(Node):
    def __init__(self):
        super().__init__('aruco_gripper_controller')
        
        # Parameters
        self.declare_parameter('target_pick_id', 101)
        self.declare_parameter('target_place_id', 102) 
        self.declare_parameter('distance_threshold', 10.0)  # cm
        self.declare_parameter('action_cooldown', 3.0)     # seconds between actions
        
        self.target_pick_id = self.get_parameter('target_pick_id').get_parameter_value().integer_value
        self.target_place_id = self.get_parameter('target_place_id').get_parameter_value().integer_value
        self.distance_threshold = self.get_parameter('distance_threshold').get_parameter_value().double_value
        self.action_cooldown = self.get_parameter('action_cooldown').get_parameter_value().double_value
        
        # State tracking
        self.last_grip_time = 0
        self.last_release_time = 0
        self.gripper_state = 0  # 0=unknown, 1=gripped, 2=released
        
        # Subscribe to specific marker position topics
        self.pick_marker_sub = self.create_subscription(
            Point,
            f'/aruco/marker_{self.target_pick_id}/position',
            self.pick_marker_callback,
            10
        )
        
        self.place_marker_sub = self.create_subscription(
            Point,
            f'/aruco/marker_{self.target_place_id}/position',
            self.place_marker_callback,
            10
        )
        
        # Optional: Subscribe to other markers for monitoring
        self.marker121_sub = self.create_subscription(
            Point,
            '/aruco/marker_121/position',
            lambda msg: self.monitor_marker_callback(msg, 121),
            10
        )
        
        self.marker122_sub = self.create_subscription(
            Point,
            '/aruco/marker_122/position',
            lambda msg: self.monitor_marker_callback(msg, 122),
            10
        )
        
        # Publisher to arm commander (/arm/mode)
        self.arm_publisher = self.create_publisher(
            UInt8,
            '/arm/mode',
            10
        )
        
        # Service for manual gripper control
        self.gripper_service = self.create_service(
            SetBool,
            '/gripper/control',
            self.gripper_service_callback
        )
        
        self.get_logger().info('ArUco Gripper Controller started')
        self.get_logger().info(f'Pick marker ID: {self.target_pick_id}')
        self.get_logger().info(f'Place marker ID: {self.target_place_id}')
        self.get_logger().info(f'Distance threshold: {self.distance_threshold}cm')
        self.get_logger().info(f'Action cooldown: {self.action_cooldown}s')
        
    def calculate_distance_cm(self, position):
        """Calculate euclidean distance from Point message (already in cm)"""
        distance = math.sqrt(position.x**2 + position.y**2 + position.z**2)
        return distance
    
    def send_arm_command(self, mode, marker_id, distance, action_type):
        """Send command to serial arm commander"""
        current_time = time.time()
        
        # Check cooldown based on action type
        if action_type == "grip" and (current_time - self.last_grip_time) < self.action_cooldown:
            self.get_logger().info(f"Grip action on cooldown, ignoring marker {marker_id}")
            return False
            
        if action_type == "release" and (current_time - self.last_release_time) < self.action_cooldown:
            self.get_logger().info(f"Release action on cooldown, ignoring marker {marker_id}")
            return False
        
        try:
            msg = UInt8()
            msg.data = mode
            self.arm_publisher.publish(msg)
            
            # Update state and timing
            self.gripper_state = mode
            if action_type == "grip":
                self.last_grip_time = current_time
            else:
                self.last_release_time = current_time
            
            action_name = "GRIP" if mode == 1 else "RELEASE"
            self.get_logger().info(
                f'{action_name} command sent! Marker {marker_id} at {distance:.1f}cm'
            )
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to send arm command: {e}')
            return False
    
    def pick_marker_callback(self, msg):
        """Handle pick marker detection (triggers grip)"""
        distance = self.calculate_distance_cm(msg)
        
        self.get_logger().info(
            f"Pick marker {self.target_pick_id}: {distance:.1f}cm "
            f"{'[WITHIN RANGE]' if distance <= self.distance_threshold else '[TOO FAR]'}"
        )
        
        # Trigger grip if within threshold and not already gripped
        if distance <= self.distance_threshold:
            if self.gripper_state != 1:  # Not already gripped
                success = self.send_arm_command(1, self.target_pick_id, distance, "grip")
                if success:
                    self.get_logger().info(f"*** PICK ACTION TRIGGERED BY MARKER {self.target_pick_id} ***")
    
    def place_marker_callback(self, msg):
        """Handle place marker detection (triggers release)"""
        distance = self.calculate_distance_cm(msg)
        
        self.get_logger().info(
            f"Place marker {self.target_place_id}: {distance:.1f}cm "
            f"{'[WITHIN RANGE]' if distance <= self.distance_threshold else '[TOO FAR]'}"
        )
        
        # Trigger release if within threshold and not already released
        if distance <= self.distance_threshold:
            if self.gripper_state != 2:  # Not already released
                success = self.send_arm_command(2, self.target_place_id, distance, "release")
                if success:
                    self.get_logger().info(f"*** PLACE ACTION TRIGGERED BY MARKER {self.target_place_id} ***")
    
    def monitor_marker_callback(self, msg, marker_id):
        """Monitor other markers for logging purposes"""
        distance = self.calculate_distance_cm(msg)
        self.get_logger().info(f"Monitor marker {marker_id}: {distance:.1f}cm")
    
    def gripper_service_callback(self, request, response):
        """ROS2 service for manual gripper control"""
        try:
            if request.data:  # True = grip
                msg = UInt8()
                msg.data = 1
                self.arm_publisher.publish(msg)
                self.gripper_state = 1
                response.success = True
                response.message = "Manual GRIP command sent to arm"
                self.get_logger().info("Manual GRIP command executed")
                
            else:  # False = release
                msg = UInt8()
                msg.data = 2
                self.arm_publisher.publish(msg)
                self.gripper_state = 2
                response.success = True
                response.message = "Manual RELEASE command sent to arm"
                self.get_logger().info("Manual RELEASE command executed")
                
            # Reset cooldowns for manual commands
            current_time = time.time()
            self.last_grip_time = current_time if request.data else self.last_grip_time
            self.last_release_time = current_time if not request.data else self.last_release_time
                
        except Exception as e:
            response.success = False
            response.message = f"Failed to control gripper: {str(e)}"
            self.get_logger().error(f"Manual gripper control failed: {e}")
        
        return response

def main(args=None):
    rclpy.init(args=args)
    controller = ArucoGripperController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("\nShutting down ArUco Gripper Controller...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
