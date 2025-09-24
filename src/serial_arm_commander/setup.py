from setuptools import find_packages, setup

package_name = 'serial_arm_commander'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/arm_commander_launch.py']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='ired',
    maintainer_email='ired@todo.todo',
    description='ROS2 node that forwards /arm/mode to Arduino via serial',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'serial_node = serial_arm_commander.serial_node:main',
            'aruco_gripper_controller = serial_arm_commander.aruco_gripper_controller:main'
        ],
    },
)
