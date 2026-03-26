from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, EqualsSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    real_robot_arg = DeclareLaunchArgument(
        name='real_robot',
        default_value='true',
        description='Control real robot (true/false)'
    )

    enable_hand_control_arg = DeclareLaunchArgument(
        name='enable_hand_control',
        default_value='true',
        description='Enable hand control (true/false)'
    )

    init_delay_arg = DeclareLaunchArgument(
        name='init_delay',
        default_value='5.0',
        description='Delay before initialization in seconds'
    )

    end_effector_arg = DeclareLaunchArgument(
        name='end_effector',
        default_value='hand',  # hand / gripper
        description='End effector type: hand or gripper'
    )

    inspire_hand_node = ExecuteProcess(
        cmd=['ros2', 'run', 'inspire_hand', 'inspire_hand_sub'],
        output='screen',
        name='inspire_hand_node',
        condition=IfCondition(
            EqualsSubstitution(
                LaunchConfiguration('end_effector'), 'hand'
            )
        )
    )

    hand_grasp_state_pub_node = ExecuteProcess(
        cmd=['ros2', 'run', 'inspire_hand', 'hand_grasp_state_pub'],
        output='screen',
        name='hand_grasp_state_pub',
        condition=IfCondition(
            EqualsSubstitution(
                LaunchConfiguration('end_effector'), 'hand'
            )
        )
    )

    zhixing_gripper_node = ExecuteProcess(
        cmd=['ros2', 'run', 'zhixing_gripper', 'gripper'],
        output='screen',
        name='zhixing_gripper_node',
        condition=IfCondition(
            EqualsSubstitution(
                LaunchConfiguration('end_effector'), 'gripper'
            )
        )
    )

    head_camera_launch = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'start_robot',
            'gemini2L_recorder_launch.py'
        ],
        output='log',
        name='head_camera_launch'
    )

    set_head_camera_params = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'set -e; '
            'echo "[head_cam] waiting for /top/top/set_color_exposure ..."; '
            'until ros2 service type /top/top/set_color_exposure >/dev/null 2>&1; do sleep 0.2; done; '
            'ros2 service call /top/top/set_color_auto_exposure std_srvs/srv/SetBool "{data: false}"; '
            'ros2 service call /top/top/set_color_exposure orbbec_camera_msgs/srv/SetInt32 "{data: 8000}"; '
            'ros2 service call /top/top/set_color_gain orbbec_camera_msgs/srv/SetInt32 "{data: 10}"; '
            'ros2 service call /top/top/get_color_exposure orbbec_camera_msgs/srv/GetInt32 "{}"; '
            'ros2 service call /top/top/get_color_gain orbbec_camera_msgs/srv/GetInt32 "{}"; '
        ],
        output='screen',
        name='set_head_camera_params'
    )

    head_cam_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=head_camera_launch,
            on_start=[
                TimerAction(
                    period=6.0,
                    actions=[set_head_camera_params]
                )
            ]
        )
    )

    return LaunchDescription([

        # 参数声明
        real_robot_arg,
        enable_hand_control_arg,
        init_delay_arg,
        end_effector_arg,

        # 启动硬件及控制器（带参数传递）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ymbot_d_control'),
                    'launch/activate_multiple_groups.launch.py'
                ])
            ]),
            launch_arguments={
                'real_robot': LaunchConfiguration('real_robot')
                # 使用动态参数
            }.items()
        ),

        # 初始化关节位置
        TimerAction(
            period=LaunchConfiguration('init_delay'),
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2',
                        'run',
                        'remote_operate_pkg',
                        'joint_control',
                        '--ros-args',
                        '-p',
                        # 参考src/remote_operate_pkg/remote_operate_pkg/joint_control.py:17
                        # leftarm rightarm
                        'target_positions:=[-0.35, 0.0, 0.0, -0.52, 0.0, 0.0, 0.0, '
                        '0.35, 0.0, 0.0, 0.52, 0.0, 0.0, 0.0, 0.0, -0.59]'
                    ],
                    output='screen',
                    name='joint_initialization'
                )
            ]
        ),

        # 初始化手部位置（带条件判断）
        TimerAction(
            period=LaunchConfiguration('init_delay'),
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'remote_operate_pkg', 'hand_control',
                        '--ros-args',
                        '-p',
                        'left_hand_positions:=[0.0,0.0,0.0,0.0,0.0,0.0]',
                        '-p',
                        'right_hand_positions:=[0.0,0.0,0.0,0.0,0.0,0.0]',
                        '-p',
                        'duration:=3.0'
                    ],
                    output='screen',
                    name='hand_initialization',
                    condition=IfCondition(EqualsSubstitution(LaunchConfiguration('end_effector'), 'hand'))
                )
            ]
        ),

        # 启动末端执行器节点
        inspire_hand_node,
        hand_grasp_state_pub_node,
        zhixing_gripper_node,

        # 启动底盘驱动节点
        ExecuteProcess(
            cmd=['ros2', 'run', 'chassis_driver_node', 'chassis_driver_node'],
            output='screen',
            name='chassis_driver_node'
        ),

        # 启动VR指令发送节点
        ExecuteProcess(
            cmd=['ros2', 'run', 'vr_receiver_tcp', 'vr_receiver_tcp'],
            output='screen',
            name='vr_receiver_node'
        ),

        # ik解 & 碰撞检测
        ExecuteProcess(
            cmd=['ros2', 'run', 'ik_solver_pkg', 'ik_servo_node_coll'],
            output='log'
        ),

        # # 启动腕部相机节点（保持不变）
        # ExecuteProcess(
        #     cmd=[
        #         'ros2', 'launch', 'start_robot', 'rs_multi_camera_launch.py'
        #     ],
        #     output='log'
        # ),

        # 启动头部相机节点（保持不变）
        head_camera_launch,

        # 设置头部相机曝光/增益
        head_cam_handler

    ])
