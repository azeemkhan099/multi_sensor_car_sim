from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='multi_sensor_car_sim',
            executable='sensor_publisher',
            name='multi_sensor_node',
            output='screen'
        )
    ])
