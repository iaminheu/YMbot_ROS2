#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

class NeckControl(Node):
    def __init__(self):
        super().__init__('neck_control')
        
        # 声明参数并设置默认值
        self.declare_parameter('target_positions', [0.0, 0.0])  # 默认目标位置
        self.first_receive = True
        # 订阅关节状态
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        # 发布轨迹命令
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/neck_controller/joint_trajectory',
            10)
        
        # 颈部关节名称
        self.neck_joints = ['Neck_Joint1', 'Neck_Joint2']
        self.current_positions = None
        
        # 插值参数
        self.duration = 3.0  # 轨迹总时长(秒)
        self.steps = 150     # 插值步数
        
        self.get_logger().info("Neck Controller initialized - waiting for initial joint states...")

    def joint_state_callback(self, msg):
        # 如果是第一次收到消息，记录当前位置
        if any(pos is None for pos in msg.position):
            self.get_logger().warn("Received joint states with None values, skipping...")
            return
            # 检查是否包含颈部关节
        if self.first_receive:
            # 创建名称到位置的映射
            self.name_to_pos = {name: pos for name, pos in zip(msg.name, msg.position)}
            self.first_receive = False
            
        if all(joint in self.name_to_pos for joint in self.neck_joints):
            self.current_positions = [self.name_to_pos[joint] for joint in self.neck_joints]
            self.get_logger().info(f"Initial positions captured: {dict(zip(self.neck_joints, self.current_positions))}")
            
            # 获取参数值
            target_positions = self.get_parameter('target_positions').get_parameter_value().double_array_value
            self.get_logger().info(f"Target positions from parameters: {target_positions}")
            
            # 生成并发送轨迹
            self.send_neck_trajectory(target_positions)
            
            # 取消订阅
            # self.destroy_subscription(self.subscription)
            
            # 设置定时器在轨迹完成后关闭节点
            self.shutdown_timer = self.create_timer(
                self.duration + 1.0,  # 等待轨迹完成时间+1秒缓冲
                self.shutdown_node
            )
        else:
            self.get_logger().warn("Not all neck joints found in /joint_states")
            self.shutdown_node()

    def send_neck_trajectory(self, target_positions):
        if self.current_positions is None:
            self.get_logger().error("No current positions available!")
            self.shutdown_node()
            return
        
        # 创建轨迹消息
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.neck_joints
        
        # 线性插值
        for i in range(self.steps + 1):
            point = JointTrajectoryPoint()
            
            # 计算插值比例 (0.0到1.0)
            alpha = i / self.steps
            
            # 计算插值位置
            interpolated_pos = [
                start + alpha * (target - start)
                for start, target in zip(self.current_positions, target_positions)
            ]
            
            point.positions = interpolated_pos
            
            # 设置时间戳(均匀分布)
            point.time_from_start = rclpy.time.Duration(seconds=self.duration * alpha).to_msg()
            
            trajectory_msg.points.append(point)
        
        # 发布轨迹
        self.publisher.publish(trajectory_msg)
        self.get_logger().info(
            f"Published neck trajectory from {self.current_positions} to {target_positions} "
            f"over {self.duration} seconds with {self.steps} steps"
        )

    def shutdown_node(self):
        self.get_logger().info("Trajectory completed, shutting down node...")
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    neck_control = NeckControl()
    try:
        rclpy.spin(neck_control)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            neck_control.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()