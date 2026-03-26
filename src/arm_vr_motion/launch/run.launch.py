from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():


    return LaunchDescription([
        # 1. 启动灵巧手节点（保持不变）
        ExecuteProcess(
            cmd=['ros2', 'run', 'ymbot_d_control', 'body_joint_tracking'],
            output='screen'
        ),

    ])