from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('laser_rviz'),
        'config',
        'laser_view.rviz'
    )

    # laser_filter_config = os.path.join(
    #     get_package_share_directory('laser_rviz'),
    #     'config',
    #     'laser_filter_chain.yaml'
    # )

    return LaunchDescription([
        Node( # rviz
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])
