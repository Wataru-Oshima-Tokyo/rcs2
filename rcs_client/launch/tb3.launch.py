
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level: debug, info, warning, error, fatal"
    )

    rcs_client_node = Node(
        package='rcs_client',
        executable='rcs_client_node',
        output='screen',
    )

    tb3_rcs_proxy = Node(
        package='rcs_client',
        executable='turtlebot3_rcs_proxy',
        output='screen',
    )

    move_base_proxy = Node(
        package='rcs_client',
        executable='move_base_proxy',
        output='screen',
        parameters=[
            {'map_frame': 'map'},
            {'robot_frame': 'base_link'},
            {'world_frame': 'map'},
        ]
    )


    return LaunchDescription([
        rcs_client_node,
        tb3_rcs_proxy,
        move_base_proxy,
    ])
