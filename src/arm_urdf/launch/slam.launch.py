from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                os.path.join(get_package_share_directory('arm_urdf'), 'config', 'slam.yaml')
            ],
            output='screen'
        )

    ])
