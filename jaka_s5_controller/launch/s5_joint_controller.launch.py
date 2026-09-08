from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "jaka_s5", package_name="jaka_s5_moveit_config"
        ).to_moveit_configs()
    )
    s5_config_share = get_package_share_directory("jaka_s5_moveit_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "ip", default_value="10.5.5.100", description="JAKA robot IP address"
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="true", description="Start RViz"
        ),
        DeclareLaunchArgument(
            "diagnostics_enabled",
            default_value="false",
            description="Publish JAKA controller diagnostic state",
        ),
        DeclareLaunchArgument(
            "auto_power_off_on_exit",
            default_value="true",
            description="Disable and power off the robot when the controller exits",
        ),
        DeclareLaunchArgument(
            "auto_move_to_initial_on_start",
            default_value="false",
            description="Move to the S5 initial pose once after startup",
        ),
        Node(
            package="jaka_s5_controller",
            executable="s5_moveit_server",
            name="moveit_server",
            output="screen",
            parameters=[{
                "ip": LaunchConfiguration("ip"),
                "model": "s5",
                "diagnostics_enabled": LaunchConfiguration("diagnostics_enabled"),
                "auto_power_off_on_exit": LaunchConfiguration("auto_power_off_on_exit"),
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(s5_config_share, "launch", "demo.launch.py")
            ),
            launch_arguments={
                "use_rviz_sim": "false",
                "use_rviz": LaunchConfiguration("use_rviz"),
            }.items(),
        ),
        Node(
            package="jaka_s5_controller",
            executable="s5_joint_controller",
            name="s5_joint_controller",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {
                    "auto_power_off_on_exit": LaunchConfiguration("auto_power_off_on_exit"),
                    "auto_move_to_initial_on_start": LaunchConfiguration(
                        "auto_move_to_initial_on_start"
                    ),
                },
            ],
        ),
    ])
