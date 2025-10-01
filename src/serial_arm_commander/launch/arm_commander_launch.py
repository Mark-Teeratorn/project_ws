# launch/arm_commander_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serial_arm_commander',
            executable='serial_node',
            name='serial_arm_commander',
            output='screen',
            parameters=[
                {'port': '/dev/ttyARM'},   # change if your board appears as /dev/ttyUSB0
                {'baud_rate': 115200}
            ]
        ),
        
        Node(
            package='serial_arm_commander',
            executable='aruco_gripper_controller',
            name='aruco_gripper_controller',
            output='screen'
        )
    ])