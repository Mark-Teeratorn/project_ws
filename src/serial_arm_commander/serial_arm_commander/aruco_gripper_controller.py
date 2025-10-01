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
        
        # Parameters - Two sets of pick/place operations
        self.declare_parameter('target_pick_1', 101)
        self.declare_parameter('target_place_1', 102)
        self.declare_parameter('target_pick_2', 121) 
        self.declare_parameter('target_place_2', 122)
        self.declare_parameter('distance_threshold', 14.0)  # cm
        self.declare_parameter('action_cooldown', 3.0)     # seconds between actions
        
        self.target_pick_1 = self.get_parameter('target_pick_1').get_parameter_value().integer_value
        self.target_place_1 = self.get_parameter('target_place_1').get_parameter_value().integer_value
        self.target_pick_2 = self.get_parameter('target_pick_2').get_parameter_value().integer_value
        self.target_place_2 = self.get_parameter('target_place_2').get_parameter_value().integer_value
        self.distance_threshold = self.get_parameter('distance_threshold').get_parameter_value().double_value
        self.action_cooldown = self.get_parameter('action_cooldown').get_parameter_value().double_value
        
        # Enhanced state tracking
        self.last_grip_time = 0
        self.last_release_time = 0
        self.gripper_state = 0  # 0=unknown/initial, 1=gripped, 2=released
        
        # Track processed markers - reset based on gripper state changes
        self.processed_pick_markers = set()  # Reset when gripper releases
        self.processed_place_markers = set()  # Reset when gripper grips
        
        # Sequential workflow control
        self.current_sequence = 1  # 1=target1, 2=target2
        self.sequence_step = "pick"  # "pick" or "place"
        self.target1_completed = False
        self.target2_completed = False
        
        # Subscribe to specific marker position topics - Set 1
        self.pick1_marker_sub = self.create_subscription(
            Point,
            f'/aruco/marker_{self.target_pick_1}/position',
            self.pick1_marker_callback,
            10
        )
        
        self.place1_marker_sub = self.create_subscription(
            Point,
            f'/aruco/marker_{self.target_place_1}/position',
            self.place1_marker_callback,
            10
        )
        
        # Subscribe to specific marker position topics - Set 2
        self.pick2_marker_sub = self.create_subscription(
            Point,
            f'/aruco/marker_{self.target_pick_2}/position',
            self.pick2_marker_callback,
            10
        )
        
        self.place2_marker_sub = self.create_subscription(
            Point,
            f'/aruco/marker_{self.target_place_2}/position',
            self.place2_marker_callback,
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
        
        self.get_logger().info('ArUco Gripper Controller started - Sequential Pick/Place System')
        self.get_logger().info(f'Target 1: Pick {self.target_pick_1} → Place {self.target_place_1}')
        self.get_logger().info(f'Target 2: Pick {self.target_pick_2} → Place {self.target_place_2}')
        self.get_logger().info(f'Distance threshold: {self.distance_threshold}cm')
        self.get_logger().info(f'Action cooldown: {self.action_cooldown}s')
        self.get_logger().info(f'Initial gripper state: {self.get_state_name()}')
        self.get_logger().info(f'Current sequence: Target {self.current_sequence} - {self.sequence_step.upper()}')
        
    def get_state_name(self):
        """Get human-readable state name"""
        state_names = {0: "UNKNOWN", 1: "GRIPPED", 2: "RELEASED"}
        return state_names.get(self.gripper_state, "INVALID")
        
    def can_grip(self):
        """Check if gripper can perform grip action"""
        return self.gripper_state != 1  # Can grip if not already gripped
        
    def can_release(self):
        """Check if gripper can perform release action"""
        return self.gripper_state != 2  # Can release if not already released
        
    def get_distance_from_point(self, position):
        """Get distance from Point message (ArUco monitor already provides cm values)"""
        # The ArUco monitor already converted to cm, so just calculate distance
        distance = math.sqrt(position.x**2 + position.y**2 + position.z**2)
        return distance
    
    def get_sequence_status(self):
        """Get current sequence status for logging"""
        return f"Target {self.current_sequence} - {self.sequence_step.upper()}"
    
    def is_current_target_active(self, target_num):
        """Check if the given target is currently active in sequence"""
        return self.current_sequence == target_num
    
    def advance_sequence(self):
        """Advance to next step in sequence"""
        if self.sequence_step == "pick":
            # Move to place step for same target
            self.sequence_step = "place"
            self.get_logger().info(f"Sequence advanced: Target {self.current_sequence} - PLACE")
            
        elif self.sequence_step == "place":
            if self.current_sequence == 1:
                # Target 1 completed, move to target 2
                self.target1_completed = True
                self.current_sequence = 2
                self.sequence_step = "pick"
                self.get_logger().info("*** TARGET 1 COMPLETED! Moving to Target 2 - PICK ***")
                
            elif self.current_sequence == 2:
                # Target 2 completed, sequence finished
                self.target2_completed = True
                self.get_logger().info("*** ALL TARGETS COMPLETED! Sequence finished ***")
                # Could reset sequence here or keep finished state
                
    def reset_sequence(self):
        """Reset sequence to beginning (can be called via service)"""
        self.current_sequence = 1
        self.sequence_step = "pick"
        self.target1_completed = False
        self.target2_completed = False
        self.processed_pick_markers.clear()
        self.processed_place_markers.clear()
        self.get_logger().info("*** SEQUENCE RESET: Starting from Target 1 - PICK ***")
    
    def is_pick_marker_processed(self, marker_id):
        """Check if pick marker was already processed (until release)"""
        return marker_id in self.processed_pick_markers
    
    def is_place_marker_processed(self, marker_id):
        """Check if place marker was already processed (until grip)"""
        return marker_id in self.processed_place_markers
    
    def mark_pick_marker_processed(self, marker_id):
        """Mark pick marker as processed (will be reset on release)"""
        self.processed_pick_markers.add(marker_id)
        self.get_logger().info(f"Pick marker {marker_id} marked as processed until release")
    
    def mark_place_marker_processed(self, marker_id):
        """Mark place marker as processed (will be reset on grip)"""
        self.processed_place_markers.add(marker_id)
        self.get_logger().info(f"Place marker {marker_id} marked as processed until grip")
    
    def reset_processed_markers_on_state_change(self, new_state):
        """Reset processed markers based on gripper state changes and advance sequence"""
        if new_state == 1:  # GRIPPED - reset place markers and advance sequence
            if self.processed_place_markers:
                self.get_logger().info(f"GRIPPED: Resetting processed place markers: {self.processed_place_markers}")
                self.processed_place_markers.clear()
            self.advance_sequence()  # Move from pick to place
                
        elif new_state == 2:  # RELEASED - reset pick markers and advance sequence
            if self.processed_pick_markers:
                self.get_logger().info(f"RELEASED: Resetting processed pick markers: {self.processed_pick_markers}")
                self.processed_pick_markers.clear()
            self.advance_sequence()  # Move to next target or complete
    
    def send_arm_command(self, mode, marker_id, distance, action_type):
        """Send command to serial arm commander with state validation"""
        current_time = time.time()
        
        # State validation - prevent repetitive actions
        if action_type == "grip" and not self.can_grip():
            self.get_logger().warn(
                f"GRIP command ignored - already in {self.get_state_name()} state (marker {marker_id})"
            )
            return False
            
        if action_type == "release" and not self.can_release():
            self.get_logger().warn(
                f"RELEASE command ignored - already in {self.get_state_name()} state (marker {marker_id})"
            )
            return False
        
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
            old_state = self.get_state_name()
            self.gripper_state = mode
            new_state = self.get_state_name()
            
            # Reset processed markers based on state change
            self.reset_processed_markers_on_state_change(mode)
            
            if action_type == "grip":
                self.last_grip_time = current_time
            else:
                self.last_release_time = current_time
            
            action_name = "GRIP" if mode == 1 else "RELEASE"
            self.get_logger().info(
                f'{action_name} command sent! Marker {marker_id} at {distance:.1f}cm | '
                f'State: {old_state} → {new_state}'
            )
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to send arm command: {e}')
            return False
    
    def pick1_marker_callback(self, msg):
        """Handle pick 1 marker detection (triggers grip) - Only during Target 1 PICK sequence"""
        # ArUco monitor already provides distance in cm
        distance = self.get_distance_from_point(msg)
        
        self.get_logger().info(
            f"Pick 1 marker {self.target_pick_1}: {distance:.1f}cm "
            f"{'[WITHIN RANGE]' if distance <= self.distance_threshold else '[TOO FAR]'} "
            f"State: {self.get_state_name()} | Sequence: {self.get_sequence_status()}"
        )
        
        # Check sequence - only allow during Target 1 PICK
        if not (self.is_current_target_active(1) and self.sequence_step == "pick"):
            self.get_logger().info(f"Pick 1 marker ignored - not Target 1 PICK phase (current: {self.get_sequence_status()})")
            return
        
        # Check if already processed this pick marker (until release)
        if self.is_pick_marker_processed(self.target_pick_1):
            self.get_logger().info(f"Pick 1 marker {self.target_pick_1} already processed - ignoring until release")
            return
        
        # Trigger grip if within threshold and can grip
        if distance <= self.distance_threshold:
            if self.can_grip():
                success = self.send_arm_command(1, self.target_pick_1, distance, "grip")
                if success:
                    self.mark_pick_marker_processed(self.target_pick_1)
                    self.get_logger().info(f"*** TARGET 1 PICK ACTION TRIGGERED BY MARKER {self.target_pick_1} ***")
            else:
                self.get_logger().info(
                    f"Pick 1 marker detected but gripper already {self.get_state_name()} - action ignored"
                )
    
    def place1_marker_callback(self, msg):
        """Handle place 1 marker detection (triggers release) - Only during Target 1 PLACE sequence"""
        # ArUco monitor already provides distance in cm
        distance = self.get_distance_from_point(msg)
        
        self.get_logger().info(
            f"Place 1 marker {self.target_place_1}: {distance:.1f}cm "
            f"{'[WITHIN RANGE]' if distance <= self.distance_threshold else '[TOO FAR]'} "
            f"State: {self.get_state_name()} | Sequence: {self.get_sequence_status()}"
        )
        
        # Check sequence - only allow during Target 1 PLACE
        if not (self.is_current_target_active(1) and self.sequence_step == "place"):
            self.get_logger().info(f"Place 1 marker ignored - not Target 1 PLACE phase (current: {self.get_sequence_status()})")
            return
        
        # Check if already processed this place marker (until grip)
        if self.is_place_marker_processed(self.target_place_1):
            self.get_logger().info(f"Place 1 marker {self.target_place_1} already processed - ignoring until grip")
            return
        
        # Trigger release if within threshold and can release
        if distance <= self.distance_threshold:
            if self.can_release():
                success = self.send_arm_command(2, self.target_place_1, distance, "release")
                if success:
                    self.mark_place_marker_processed(self.target_place_1)
                    self.get_logger().info(f"*** TARGET 1 PLACE ACTION TRIGGERED BY MARKER {self.target_place_1} ***")
            else:
                self.get_logger().info(
                    f"Place 1 marker detected but gripper already {self.get_state_name()} - action ignored"
                )
    
    def pick2_marker_callback(self, msg):
        """Handle pick 2 marker detection (triggers grip) - Only during Target 2 PICK sequence"""
        # ArUco monitor already provides distance in cm
        distance = self.get_distance_from_point(msg)
        
        self.get_logger().info(
            f"Pick 2 marker {self.target_pick_2}: {distance:.1f}cm "
            f"{'[WITHIN RANGE]' if distance <= self.distance_threshold else '[TOO FAR]'} "
            f"State: {self.get_state_name()} | Sequence: {self.get_sequence_status()}"
        )
        
        # Check sequence - only allow during Target 2 PICK
        if not (self.is_current_target_active(2) and self.sequence_step == "pick"):
            self.get_logger().info(f"Pick 2 marker ignored - not Target 2 PICK phase (current: {self.get_sequence_status()})")
            return
        
        # Check if already processed this pick marker (until release)
        if self.is_pick_marker_processed(self.target_pick_2):
            self.get_logger().info(f"Pick 2 marker {self.target_pick_2} already processed - ignoring until release")
            return
        
        # Trigger grip if within threshold and can grip
        if distance <= self.distance_threshold:
            if self.can_grip():
                success = self.send_arm_command(1, self.target_pick_2, distance, "grip")
                if success:
                    self.mark_pick_marker_processed(self.target_pick_2)
                    self.get_logger().info(f"*** TARGET 2 PICK ACTION TRIGGERED BY MARKER {self.target_pick_2} ***")
            else:
                self.get_logger().info(
                    f"Pick 2 marker detected but gripper already {self.get_state_name()} - action ignored"
                )
    
    def place2_marker_callback(self, msg):
        """Handle place 2 marker detection (triggers release) - Only during Target 2 PLACE sequence"""
        # ArUco monitor already provides distance in cm
        distance = self.get_distance_from_point(msg)
        
        self.get_logger().info(
            f"Place 2 marker {self.target_place_2}: {distance:.1f}cm "
            f"{'[WITHIN RANGE]' if distance <= self.distance_threshold else '[TOO FAR]'} "
            f"State: {self.get_state_name()} | Sequence: {self.get_sequence_status()}"
        )
        
        # Check sequence - only allow during Target 2 PLACE
        if not (self.is_current_target_active(2) and self.sequence_step == "place"):
            self.get_logger().info(f"Place 2 marker ignored - not Target 2 PLACE phase (current: {self.get_sequence_status()})")
            return
        
        # Check if already processed this place marker (until grip)
        if self.is_place_marker_processed(self.target_place_2):
            self.get_logger().info(f"Place 2 marker {self.target_place_2} already processed - ignoring until grip")
            return
        
        # Trigger release if within threshold and can release
        if distance <= self.distance_threshold:
            if self.can_release():
                success = self.send_arm_command(2, self.target_place_2, distance, "release")
                if success:
                    self.mark_place_marker_processed(self.target_place_2)
                    self.get_logger().info(f"*** TARGET 2 PLACE ACTION TRIGGERED BY MARKER {self.target_place_2} ***")
            else:
                self.get_logger().info(
                    f"Place 2 marker detected but gripper already {self.get_state_name()} - action ignored"
                )
    
    def gripper_service_callback(self, request, response):
        """ROS2 service for manual gripper control with state validation"""
        try:
            current_time = time.time()
            
            if request.data:  # True = grip
                if not self.can_grip():
                    response.success = False
                    response.message = f"Cannot GRIP - gripper already {self.get_state_name()}"
                    self.get_logger().warn(f"Manual GRIP rejected - already {self.get_state_name()}")
                    return response
                
                msg = UInt8()
                msg.data = 1
                self.arm_publisher.publish(msg)
                
                old_state = self.get_state_name()
                self.gripper_state = 1
                self.last_grip_time = current_time
                
                # Reset processed markers on state change
                self.reset_processed_markers_on_state_change(1)
                
                response.success = True
                response.message = f"Manual GRIP executed | State: {old_state} → {self.get_state_name()}"
                self.get_logger().info(f"Manual GRIP executed | State: {old_state} → GRIPPED")
                
            else:  # False = release
                if not self.can_release():
                    response.success = False
                    response.message = f"Cannot RELEASE - gripper already {self.get_state_name()}"
                    self.get_logger().warn(f"Manual RELEASE rejected - already {self.get_state_name()}")
                    return response
                
                msg = UInt8()
                msg.data = 2
                self.arm_publisher.publish(msg)
                
                old_state = self.get_state_name()
                self.gripper_state = 2
                self.last_release_time = current_time
                
                # Reset processed markers on state change
                self.reset_processed_markers_on_state_change(2)
                
                response.success = True
                response.message = f"Manual RELEASE executed | State: {old_state} → {self.get_state_name()}"
                self.get_logger().info(f"Manual RELEASE executed | State: {old_state} → RELEASED")
                
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