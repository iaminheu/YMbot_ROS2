from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
import os
import launch
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # moveit_config = MoveItConfigsBuilder("ymbot_d", package_name="ymbot_d_moveit_config").to_moveit_configs()
    # return generate_demo_launch(moveit_config)
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

    # MoveIt配置
    moveit_config = MoveItConfigsBuilder("ymbot_d", package_name="ymbot_d_moveit_config").to_moveit_configs()
    moveit_launch = generate_demo_launch(moveit_config)

    return launch.LaunchDescription(
        [
            real_robot_arg,  # 添加参数声明
            driver_excutable,  # 条件启动driver_excutable
            TimerAction(period=3.0, actions=[moveit_launch])  # 延迟启动MoveIt
        ]
    )