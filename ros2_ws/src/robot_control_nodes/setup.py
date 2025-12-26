from setuptools import find_packages, setup

package_name = 'robot_control_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='ROS 2 nodes for controlling humanoid robot joints and movements',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_publisher_node = robot_control_nodes.nodes.joint_publisher_node:main',
            'joint_subscriber_node = robot_control_nodes.nodes.joint_subscriber_node:main',
            'gesture_service_node = robot_control_nodes.nodes.gesture_service_node:main',
            'gesture_client_node = robot_control_nodes.nodes.gesture_client_node:main',
            'movement_action_node = robot_control_nodes.nodes.movement_action_node:main',
        ],
    },
)