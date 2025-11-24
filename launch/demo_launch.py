from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # パッケージのパスを取得
    package_name = 'lifecycle_manager_sample'
    
    # 設定ファイルのパス
    config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'demo_params.yaml'
    )
    
    # RViz設定ファイルのパス
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'demo.rviz'
    )

    return LaunchDescription([
        # ロボットシミュレーター
        Node(
            package=package_name,
            executable='robot_simulator_node',
            name='robot_simulator_node',
            parameters=[config_file],
            output='screen'
        ),
        
        # ライフサイクルノード: straight_drive
        LifecycleNode(
            package=package_name,
            executable='straight_drive_node',
            name='straight_drive_node',
            namespace='',
            output='screen'
        ),
        
        # ライフサイクルノード: slow_drive
        LifecycleNode(
            package=package_name,
            executable='slow_drive_node',
            name='slow_drive_node',
            namespace='',
            output='screen'
        ),
        
        # ライフサイクルノード: stop
        LifecycleNode(
            package=package_name,
            executable='stop_node',
            name='stop_node',
            namespace='',
            output='screen'
        ),
        
        # ローカルプランニングマネージャー（デモマネージャーの代わりに使用）
        Node(
            package=package_name,
            executable='local_planning_manager_node',
            name='local_planning_manager_node',
            parameters=[config_file],
            output='screen'
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
    ])
