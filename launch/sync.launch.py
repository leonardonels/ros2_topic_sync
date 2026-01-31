from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ros2_topic_sync'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='ros2_topic_sync',
            executable='topic_synchronizer',
            name='topic_synchronizer',
            parameters=[config],
            output='screen'
        )
    ])
