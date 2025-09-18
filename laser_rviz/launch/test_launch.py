from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

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


def get_config(pkg_name, yaml_file):
    return os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        yaml_file
    )


def generate_launch_description():
    _pkg = 'laser_rviz'

    box_filter_cfg = get_config(_pkg, 'box_filter.yaml')
    footprint_filter_cfg = get_config(_pkg, 'footprint_filter.yaml')
    intensity_filter_cfg = get_config(_pkg, 'intensity_filter.yaml')
    shadow_filter_cfg = get_config(_pkg, 'shadow_filter.yaml')

    return LaunchDescription([
        make_filter_node('box', box_filter_cfg),

        # only use in base_link
        Node( # static_transform_publisher
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
        ),
        make_filter_node('footprint', footprint_filter_cfg),

        # Hokuyo UST-10LX not support intensity
        make_filter_node('intensity', get_config(_pkg, 'intensity_filter.yaml')),

        make_filter_node('shadow', shadow_filter_cfg),

    ])
