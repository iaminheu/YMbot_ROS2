#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明可配置参数
    real_robot_arg = DeclareLaunchArgument(
        name='real_robot',
        default_value='false',  # 设置默认值为false
        description='Control real robot (true/false)'
    )

    # 声明手部控制参数
    enable_hand_control_arg = DeclareLaunchArgument(
        name='enable_hand_control',
        default_value='true',
        description='Enable hand control (true/false)'
    )

    # 声明初始化参数
    init_delay_arg = DeclareLaunchArgument(
        name='init_delay',
        default_value='5.0',
        description='Delay before initialization in seconds'
    )

    return LaunchDescription([
        # 参数声明
        real_robot_arg,
        enable_hand_control_arg,
        init_delay_arg,

        # 1. 启动硬件及控制器（带参数传递）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ymbot_d_control'),
                    'launch/activate_multiple_groups.launch.py'
                ])
            ]),
            launch_arguments={
                'real_robot': LaunchConfiguration('real_robot')  # 使用动态参数
            }.items()
        ),

        # 2. 启动灵巧手节点（inspire_hand）
        ExecuteProcess(
            cmd=['ros2', 'run', 'inspire_hand', 'inspire_hand_sub'],
            output='screen',
            name='inspire_hand_node'
        ),

        # 3. 启动VR指令发送节点 ros2 run vr_receiver_tcp vr_receiver_tcp
        ExecuteProcess(
            cmd=['ros2', 'run', 'vr_receiver_tcp', 'vr_receiver_tcp'],
            output='screen',
            name='vr_receiver_node'
        ),

        # 4. 初始化关节位置
        TimerAction(
            period=5.0,  # 固定为5秒延迟
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'remote_operate_pkg', 'joint_control',
                         '--ros-args', '-p',
                         # 参考src/remote_operate_pkg/remote_operate_pkg/joint_control.py:17
                         #                   leftarm                               rightarm
                         'target_positions:=[-0.35, 0.0, 0.0, -0.52, 0.0, 0.0, 0.0,  0.35, 0.0, 0.0, 0.52, 0.0, 0.0, 0.0, 0.0, -0.87]'],
                    output='screen',
                    name='joint_initialization'
                )
            ]
        ),

        # # 5. 初始化颈部位置
        # TimerAction(
        #     period=5.0,  # 固定为5秒延迟
        #     actions=[
        #         ExecuteProcess(
        #             cmd=[
        #                 'ros2', 'run', 'remote_operate_pkg', 'neck_control',
        #                 '--ros-args',
        #                 '-p', 'target_positions:=[0.0,-0.5]'  # 设置颈部向下看的初始位置
        #             ],
        #             output='screen',
        #             name='neck_initialization'
        #         )
        #     ]
        # ),

        # 6. 初始化手部位置（带条件判断）
        TimerAction(
            period=5.0,  # 固定为5秒延迟
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'remote_operate_pkg', 'hand_control',
                        '--ros-args',
                        '-p', 'left_hand_positions:=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
                        '-p', 'right_hand_positions:=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
                        '-p', 'duration:=3.0'
                    ],
                    output='screen',
                    name='hand_initialization',
                    condition=IfCondition(LaunchConfiguration('enable_hand_control'))  # 修复：使用IfCondition
                )
            ]
        ),
    ])