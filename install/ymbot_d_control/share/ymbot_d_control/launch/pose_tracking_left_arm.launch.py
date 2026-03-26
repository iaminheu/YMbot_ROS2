import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ymbot_d")
        .robot_description(file_path="config/ymbot_d.urdf.xacro")
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_kinematics()
        .to_moveit_configs()
    )

    # Launch Servo as a standalone node or as a "node component" for better latency/efficiency
    launch_as_standalone_node = LaunchConfiguration(
        "launch_as_standalone_node", default="false"
    )

    # Get parameters for the Servo node
    servo_params_left_arm = {
        "moveit_servo": ParameterBuilder("ymbot_d_control")
        .yaml("config/ymbot_d_simulated_config_left_arm.yaml")
        .to_dict()
    }

    # This sets the update rate and planning group name for the acceleration limiting filter.
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name_left_arm = {"planning_group_name": "left_arm"}

    # # RViz
    # rviz_config_file = (
    #     get_package_share_directory("ymbot_d_control")
    #     + "/config/demo_rviz_config.rviz"
    # )
    # rviz_node = launch_ros.actions.Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    #     parameters=[
    #         moveit_config.robot_description,
    #         moveit_config.robot_description_semantic,
    #     ],
    # )



#########################################################################################
    # Declare the use_rviz argument with a default value of 'true'
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='Whether to start RViz'
    )

    # Get the RViz config file path
    rviz_config_file = (
        get_package_share_directory("ymbot_d_control")
        + "/config/demo_rviz_config.rviz"
    )

    # Start RViz node if use_rviz is true
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_rviz')),
    )
######################################################################################### 


    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("ymbot_d_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    controller_spawner_left_arm = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "-c", "/controller_manager"],
    )

    # Launch as much as possible in components
    container = launch_ros.actions.ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Launching as a node component makes ROS 2 intraprocess communication more efficient.
            launch_ros.descriptions.ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node_left_arm",
                parameters=[
                    servo_params_left_arm,
                    acceleration_filter_update_period,
                    planning_group_name_left_arm,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                ],
                condition=UnlessCondition(launch_as_standalone_node),
            ),
            launch_ros.descriptions.ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
            # launch_ros.descriptions.ComposableNode(
            #     package="tf2_ros",
            #     plugin="tf2_ros::StaticTransformBroadcasterNode",
            #     name="static_tf2_broadcaster",
            #     parameters=[{"child_frame_id": "/panda_link0", "frame_id": "/world"}],
            # ),
        ],
        output="screen",
    )
    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = launch_ros.actions.Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=[
            servo_params_left_arm,
            acceleration_filter_update_period,
            planning_group_name_left_arm,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
        condition=IfCondition(launch_as_standalone_node),
    )

    left_arm_control_node = launch_ros.actions.Node(
        package="ymbot_d_control",
        executable="pose_tracking_left_arm",
        name="pose_tracking_left_arm",
        output="screen",
    )

    return launch.LaunchDescription(
        [
            declare_use_rviz,
            rviz_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            controller_spawner_left_arm,
            servo_node,
            container,
            launch.actions.TimerAction(period=8.0, actions=[left_arm_control_node]),
        ]
    )
