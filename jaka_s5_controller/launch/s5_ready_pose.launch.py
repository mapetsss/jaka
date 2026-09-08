from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "jaka_s5", package_name="jaka_s5_moveit_config"
        ).to_moveit_configs()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "execute", default_value="false", description="Execute the planned ready-pose motion"
        ),
        DeclareLaunchArgument(
            "velocity_scaling", default_value="0.1", description="Velocity scaling in (0, 1]"
        ),
        DeclareLaunchArgument(
            "acceleration_scaling",
            default_value="0.1",
            description="Acceleration scaling in (0, 1]",
        ),
        DeclareLaunchArgument(
            "planning_time", default_value="10.0", description="MoveIt planning timeout"
        ),
        Node(
            package="jaka_s5_controller",
            executable="s5_ready_pose",
            name="s5_ready_pose",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {
                    "execute": LaunchConfiguration("execute"),
                    "velocity_scaling": LaunchConfiguration("velocity_scaling"),
                    "acceleration_scaling": LaunchConfiguration("acceleration_scaling"),
                    "planning_time": LaunchConfiguration("planning_time"),
                },
            ],
        ),
    ])
