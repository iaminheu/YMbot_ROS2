import os
 
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
 
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
 
from srdfdom.srdf import SRDF
 
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
 


def lbs_demo_launch(moveit_config, launch_package_path=None):
    """
    Launches a self contained demo
 
    launch_package_path is optional to use different launch and config packages
 
    Includes
     * static_virtual_joint_tfs
     * robot_state_publisher
     * move_group
     * moveit_rviz
     * warehouse_db (optional)
     * ros2_control_node + controller spawners
    """
    if launch_package_path == None:
        launch_package_path = moveit_config.package_path
 
    ld = LaunchDescription()
    ld.add_action(
        DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))
    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    virtual_joints_launch = (
        launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    )
 
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )
 
    # Given the published joint states, publish tf for the robot links
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/rsp.launch.py")
            ),
        )
    )
 
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/move_group.launch.py")
            ),
        )
    )
 
    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )
 
    # If database loading was enabled, start mongodb as well
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )
 
    robot_controlers = os.path.join(
        get_package_share_directory('udp2joint'),
        'config',
        'controllers.yaml'
    )
    # Fake joint driver
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_controlers],
            remappings=[
                ("/controller_manager/robot_description", "/robot_description"),
            ],
        )
    )
 
    # ld.add_action(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             str(launch_package_path / "launch/spawn_controllers.launch.py")
    #         ),
    #     )
    # )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster"
            ],
            output='screen'
        )
    )
    
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "left_arm_position_controller",
                "--param-file",
                robot_controlers
            ],
            output='screen'
        )
    )
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "right_arm_position_controller",
                "--param-file",
                robot_controlers
            ],
            output='screen'
        )
    )
 
    return ld

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ymbot_d", package_name="ymbot_d_moveit_config").to_moveit_configs()
    return lbs_demo_launch(moveit_config)