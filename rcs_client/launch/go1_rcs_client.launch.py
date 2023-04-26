import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    group_name = DeclareLaunchArgument(
        'group_name',
        default_value='amcl_wp_manager',
        description='Name of the group'
    )

    rcs_client_node = Node(
        package='rcs_client',
        executable='rcs_client_node',
        output='screen'
    )

    move_base_proxy = Node(
        package='rcs_client',
        executable='move_base_proxy',
        output='screen',
        parameters=[
            {'map_topic': 'map2'},
            {'robot_frame': 'base_link2'},
            {'world_frame': 'map2'}
        ]
    )

    rcs_go1_proxy = Node(
        package='rcs_client',
        executable='GO1_rcs_proxy',
        output='screen',
        parameters=[
            {'group_name': launch.substitutions.LaunchConfiguration('group_name')}
        ]
    )

    return LaunchDescription([
        group_name,
        rcs_client_node,
        move_base_proxy,
        rcs_go1_proxy
    ])
