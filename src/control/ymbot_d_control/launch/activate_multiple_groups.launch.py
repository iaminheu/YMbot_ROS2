
import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument, ExecuteProcess



def generate_launch_description():

    ## 2025.5.21 启动电机共享内存驱动进程
    # 定义是否启动真实机器人驱动的参数
    real_robot_arg = DeclareLaunchArgument(
        'real_robot',
        default_value='false',
        description='Whether to start the real robot driver'
    )

    # 获取参数值
    real_robot = LaunchConfiguration('real_robot')

    # 定义启动电机共享内存驱动进程
    cur_launch_dir = os.path.dirname(os.path.abspath(__file__))
    ws_root = os.path.abspath(os.path.join(cur_launch_dir, '..', '..', '..', '..', '..'))
    ymbot_d_sharedmemory_driver_path = os.path.join(ws_root, 'utils', 'ymbot_d_sharedmemory_driver')

    executable_path = os.path.join(ymbot_d_sharedmemory_driver_path, 'build/ymbot_d_eumotor_interface')
    driver_excutable = ExecuteProcess(
        cmd=[executable_path],
        output='screen',
        condition=launch.conditions.IfCondition(real_robot)  # 根据参数值决定是否启动
    )

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

    controller_spawner_neck = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["neck_controller", "-c", "/controller_manager"],
    )

    controller_spawner_left_arm = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "-c", "/controller_manager"],
    )

    controller_spawner_right_arm = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "-c", "/controller_manager"],
    )

    ## 2025.3.8 加入body 伺服控制器
    controller_spawner_body = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["body_controller", "-c", "/controller_manager"],
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
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),

        ],
        output="screen",
    )


    return launch.LaunchDescription(
        [
            
            real_robot_arg,  # 添加参数声明
            driver_excutable,
            launch.actions.TimerAction(period = 5.0, actions = 
            [
            declare_use_rviz,
            rviz_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            controller_spawner_neck,
            controller_spawner_left_arm,
            controller_spawner_right_arm,
            controller_spawner_body,    ## 2025.3.8 加入body 伺服控制器
            container,
            ]),
            
        ]
    )