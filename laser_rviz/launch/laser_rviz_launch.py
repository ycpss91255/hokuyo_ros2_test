from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def make_filter_node(filter_name, filter_cfg):
    return Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name=f'scan_filters_{filter_name}',
        namespace='filters',
        parameters=[filter_cfg],
        remappings=[('scan', '/scan'),
                    ('scan_filtered', f'scan_filtered_{filter_name}')],
        emulate_tty=True,
        output='screen'
    )

def make_included_launch(pkg_name, launch_file):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(pkg_name),
                'launch',
                launch_file
            )
        )
    )

def get_config(pkg_name, yaml_file):
    return os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        yaml_file
    )

def generate_launch_description():
    _pkg = 'laser_rviz'

    rviz_config = get_config(_pkg, 'laser_view.rviz')

    box_filter_cfg = get_config(_pkg, 'box_filter.yaml')
    footprint_filter_cfg = get_config(_pkg, 'footprint_filter.yaml')
    intensity_filter_cfg = get_config(_pkg, 'intensity_filter.yaml')
    shadow_filter_cfg = get_config(_pkg, 'shadow_filter.yaml')

    return LaunchDescription([
        make_included_launch('urg_node2', 'urg_node2.launch.py'),
        Node( # rviz
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            emulate_tty=True,
            arguments=['-d', rviz_config],
        ),
        # box filter
        make_filter_node('box', box_filter_cfg),
        # footprint filter (only use in base_link)
        Node( # static_transform_publisher
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
        ),
        make_filter_node('footprint', footprint_filter_cfg),
        # intensity filter
        # Hokuyo UST-10LX not support intensity
        make_filter_node('intensity', get_config(_pkg, 'intensity_filter.yaml')),
        # shadow filter
        make_filter_node('shadow', shadow_filter_cfg),
    ])
