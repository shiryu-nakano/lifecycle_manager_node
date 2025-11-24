from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('lifecycle_manager_sample'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='lifecycle_manager_sample',
            executable='local_planning_manager_node',
            name='local_planning_manager_node',
            parameters=[config],
            output='screen'
        )
    ])
