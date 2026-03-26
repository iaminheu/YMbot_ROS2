from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from pathlib import Path
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ymbot_d", package_name="ymbot_d_moveit_config").to_moveit_configs()
    launch_package_path = moveit_config.package_path

    ld = LaunchDescription()

    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(launch_package_path) / "launch/moveit_rviz.launch.py")
            )
        )
    )

    return ld




