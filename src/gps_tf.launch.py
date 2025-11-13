from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Create GPS Sensor TF Publisher
    start_gps_tf_publisher = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='gps_tf_publisher',
    arguments=[
        '--x', '0.1',
        '--y', '-0.1',
        '--z', '0.5',
        '--roll', '3.14',
        '--pitch', '0.0',
        '--yaw', '-1.57',
        '--frame-id', 'base_link',
        '--child-frame-id', 'gps'
    ]
    )
    # Create Launch Description
    ld = LaunchDescription()
    ld.add_action(start_gps_tf_publisher)
    return ld