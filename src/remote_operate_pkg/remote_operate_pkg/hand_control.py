#!/usr/bin/env python3
"""
YMBot灵巧手控制脚本 - 修改版
通过发布命令到inspire_hand节点来控制手部，避免直接控制冲突
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray
import numpy as np
import time


class HandControl(Node):
    def __init__(self):
        super().__init__('hand_control')
        
        # 声明参数
        self.declare_parameter('left_hand_positions', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('right_hand_positions', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('duration', 2.0)
        self.declare_parameter('control_frequency', 30.0)
        
        # 获取参数
        self.left_target = self.get_parameter('left_hand_positions').get_parameter_value().double_array_value
        self.right_target = self.get_parameter('right_hand_positions').get_parameter_value().double_array_value
        self.duration = self.get_parameter('duration').get_parameter_value().double_value
        self.control_frequency = self.get_parameter('control_frequency').get_parameter_value().double_value
        
        # 手部关节名称
        self.left_hand_joints = [
            'L_thumb_MCP_joint1',
            'L_thumb_MCP_joint2',
            'L_index_MCP_joint',
            'L_middle_MCP_joint',
            'L_ring_MCP_joint',
            'L_pinky_MCP_joint'
        ]
        
        self.right_hand_joints = [
            'R_thumb_MCP_joint1',
            'R_thumb_MCP_joint2',
            'R_index_MCP_joint',
            'R_middle_MCP_joint',
            'R_ring_MCP_joint',
            'R_pinky_MCP_joint'
        ]
        
        # 当前位置
        self.current_positions = {}
        self.initial_positions_captured = False
        
        # 订阅关节状态
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # 发布到inspire_hand节点的命令话题
        self.left_hand_cmd_pub = self.create_publisher(
            Int16MultiArray,
            '/left_hand_command',
            10
        )
        
        self.right_hand_cmd_pub = self.create_publisher(
            Int16MultiArray,
            '/right_hand_command',
            10
        )
        
        # 控制定时器
        self.control_timer = None
        self.start_time = None
        self.initial_left_positions = None
        self.initial_right_positions = None
        
        self.get_logger().info("Hand Control Node initialized (Inspire Hand Compatible)")
        self.get_logger().info(f"Left hand target: {self.left_target}")
        self.get_logger().info(f"Right hand target: {self.right_target}")
        self.get_logger().info(f"Duration: {self.duration}s, Frequency: {self.control_frequency}Hz")

    def joint_state_callback(self, msg):
        """处理关节状态反馈"""
        for i, name in enumerate(msg.name):
            if name in self.left_hand_joints or name in self.right_hand_joints:
                self.current_positions[name] = msg.position[i]
        # print(self.current_positions)
        
        if not self.initial_positions_captured:
            left_ready = all(joint in self.current_positions for joint in self.left_hand_joints)
            right_ready = all(joint in self.current_positions for joint in self.right_hand_joints)
            
            if left_ready and right_ready:
                self.initial_left_positions = [self.current_positions[j] for j in self.left_hand_joints]
                self.initial_right_positions = [self.current_positions[j] for j in self.right_hand_joints]
                
                self.get_logger().info(f"Initial left hand positions: {self.initial_left_positions}")
                self.get_logger().info(f"Initial right hand positions: {self.initial_right_positions}")
                
                self.initial_positions_captured = True
                self.start_control()

    def start_control(self):
        """开始手部控制"""
        self.start_time = time.time()
        period = 1.0 / self.control_frequency
        self.control_timer = self.create_timer(period, self.control_callback)
        self.get_logger().info("Started hand control")

    def control_callback(self):
        """控制回调函数"""
        if self.start_time is None:
            return
        
        elapsed_time = time.time() - self.start_time
        
        if elapsed_time > self.duration:
            # 发送最终位置
            self.send_hand_commands(self.left_target, self.right_target)
            
            # 停止控制
            self.control_timer.cancel()
            self.get_logger().info("Hand control completed")
            
            # 延迟1秒后关闭节点
            self.create_timer(1.0, lambda: self.shutdown_node())
            return
        
        # 计算插值因子
        t = elapsed_time / self.duration
        alpha = t * t * t * (t * (t * 6.0 - 15.0) + 10.0)
        
        # 计算当前目标位置
        left_positions = []
        for i in range(len(self.left_hand_joints)):
            pos = self.initial_left_positions[i] + alpha * (self.left_target[i] - self.initial_left_positions[i])
            left_positions.append(pos)
        
        right_positions = []
        for i in range(len(self.right_hand_joints)):
            pos = self.initial_right_positions[i] + alpha * (self.right_target[i] - self.initial_right_positions[i])
            right_positions.append(pos)
        
        # 发送命令
        self.send_hand_commands(left_positions, right_positions)

    def send_hand_commands(self, left_positions, right_positions):
        """发送手部控制命令到inspire_hand节点"""
        # 转换为inspire_hand期望的格式 (0-1000范围)
        left_msg = Int16MultiArray()
        left_msg.data = [int(pos * 1000) for pos in left_positions]
        self.left_hand_cmd_pub.publish(left_msg)
        
        right_msg = Int16MultiArray()
        right_msg.data = [int(pos * 1000) for pos in right_positions]
        self.right_hand_cmd_pub.publish(right_msg)
        
        if hasattr(self, '_control_count'):
            self._control_count += 1
        else:
            self._control_count = 0
            
        if self._control_count % 10 == 0:
            self.get_logger().debug(f"Sent hand commands - Left: {[f'{p:.3f}' for p in left_positions[:3]]}, "
                                  f"Right: {[f'{p:.3f}' for p in right_positions[:3]]}")

    def shutdown_node(self):
        """关闭节点"""
        self.get_logger().info("Shutting down hand control node")
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    hand_control = HandControl()
    
    try:
        rclpy.spin(hand_control)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            hand_control.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()