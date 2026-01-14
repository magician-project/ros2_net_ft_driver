from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='net_ft_driver',
            executable='ft_logger_main',
            name='ft_logger_main',
            output='screen',
            parameters=[{
                'topic_name': '/ati_ft_sensor/wrench_sensed',
                'rate': 500.0,
            }]
        )
    ])
