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
                {'port': '/dev/ttyUSB1'},   # change if your board appears as /dev/ttyUSB0
                {'baud_rate': 115200}
            ])
        

    ])
