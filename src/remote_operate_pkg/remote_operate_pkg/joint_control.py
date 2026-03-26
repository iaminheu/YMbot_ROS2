#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
np.set_printoptions(suppress=True, precision=2)
class JointControl(Node):
    def __init__(self):
        super().__init__('joint_control')

        # 声明参数并设置默认值
        self.declare_parameter(
            "target_positions",
            [
                0.0, # Left_Arm_Joint1
                0.0, # Left_Arm_Joint2
                0.0, # Left_Arm_Joint3
                0.0, # Left_Arm_Joint4
                0.0, # Left_Arm_Joint5
                0.0, # Left_Arm_Joint6
                0.0, # Left_Arm_Joint7
                0.0, # Right_Arm_Joint1
                0.0, # Right_Arm_Joint2
                0.0, # Right_Arm_Joint3
                0.0, # Right_Arm_Joint4
                0.0, # Right_Arm_Joint5
                0.0, # Right_Arm_Joint6
                0.0, # Right_Arm_Joint7
                0.0, # Neck_Joint1
                0.0, # Neck_Joint2
            ],
        )  # 默认目标位置
        self.first_receive = True
        # 订阅关节状态
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        # 发布轨迹命令
        self.neck_publisher = self.create_publisher(
            JointTrajectory,
            '/neck_controller/joint_trajectory',
            10)

        self.left_arm_publisher = self.create_publisher(
            JointTrajectory,
            '/left_arm_controller/joint_trajectory',
            10)

        self.right_arm_publisher = self.create_publisher(
            JointTrajectory,
            '/right_arm_controller/joint_trajectory',
            10)
        # 颈部关节名称
        self.control_joints = [
            "Left_Arm_Joint1",
            "Left_Arm_Joint2",
            "Left_Arm_Joint3",
            "Left_Arm_Joint4",
            "Left_Arm_Joint5",
            "Left_Arm_Joint6",
            "Left_Arm_Joint7",
            "Right_Arm_Joint1",
            "Right_Arm_Joint2",
            "Right_Arm_Joint3",
            "Right_Arm_Joint4",
            "Right_Arm_Joint5",
            "Right_Arm_Joint6",
            "Right_Arm_Joint7",
            "Neck_Joint1",
            "Neck_Joint2",
        ]
        self.current_positions = None

        # 插值参数
        # self.duration = 9.0  # 轨迹总时长(秒)
        # self.steps = 450     # 插值步数
        self.duration = 2  # 轨迹总时长(秒)
        self.steps = 100     # 插值步数

        self.get_logger().info("Neck Controller initialized - waiting for initial joint states...")

    def joint_state_callback(self, msg):
        # 如果是第一次收到消息，记录当前位置
        if any(pos is None for pos in msg.position):
            self.get_logger().warn("Received joint states with None values, skipping...")
            return
        # self.get_logger().info(f'name=\n{msg.name}\npos=\n{np.array(msg.position)}\n')
        # [joint_initialization-15] [INFO] [1762171348.589918438] [joint_control]: 
        # msg = sensor_msgs.msg.JointState(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1762171348, nanosec=588638921), frame_id=''), 
        # name=[
        # 'Body_Joint1'     , 'Body_Joint2'     , 'Body_Joint3'      , 'Body_Joint4'     , 
        # 'Left_Arm_Joint1' , 'Left_Arm_Joint2' , 'Left_Arm_Joint3'  , 'Left_Arm_Joint4' ,
        # 'Left_Arm_Joint5' , 'Left_Arm_Joint6' , 'Left_Arm_Joint7'  ,
        # 'Neck_Joint1'     , 'Neck_Joint2'     ,
        # 'Right_Arm_Joint1', 'Right_Arm_Joint2', 'Right_Arm_Joint3' , 'Right_Arm_Joint4', 
        # 'Right_Arm_Joint5', 'Right_Arm_Joint6', 'Right_Arm_Joint7'], 
        # position=[
        #  1.164568305015564,     2.544959306716919,    -1.0428193807601929,     0.023584866896271706,
        # -0.34284478425979614,   0.005709765013307333,  0.0066152047365903854, -0.5106238722801208,
        #  0.006231709383428097,  0.011600642465054989,  0.00680695241317153,
        #  0.021283896639943123, -1.3245924711227417,
        #  0.3554999530315399,    0.006658019032329321,  0.006231709383428097,   0.5250049829483032,
        #  0.006423457060009241,  0.0031637479551136494, 0.0009586505475454032],
        # velocity=[nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan], 
        # effort=[nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan])

        # 检查是否包含颈部关节
        if self.first_receive:
            # 创建名称到位置的映射
            self.name_to_pos = {name: pos for name, pos in zip(msg.name, msg.position)}
            self.first_receive = False
            self.current_positions = [self.name_to_pos[joint] for joint in self.control_joints]
            self.get_logger().info(f"Initial positions captured: {dict(zip(self.control_joints, self.current_positions))}")

        if all(joint in self.name_to_pos for joint in self.control_joints):

            # 获取参数值
            target_positions = self.get_parameter('target_positions').get_parameter_value().double_array_value
            # self.get_logger().info(f"Target positions from parameters: {target_positions}")

            # 生成并发送轨迹
            self.send_trajectory(target_positions)

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

    def send_trajectory(self, target_positions):
        if self.current_positions is None:
            self.get_logger().error("No current positions available!")
            self.shutdown_node()
            return

        # 定义各控制器对应的关节
        neck_joints = ['Neck_Joint1', 'Neck_Joint2']
        left_arm_joints = [f"Left_Arm_Joint{i}" for i in range(1, 8)]  # Joint1-Joint7
        right_arm_joints = [f"Right_Arm_Joint{i}" for i in range(1, 8)]  # Joint1-Joint7

        # 创建各控制器的轨迹消息
        neck_trajectory = JointTrajectory()
        neck_trajectory.joint_names = neck_joints

        left_arm_trajectory = JointTrajectory()
        left_arm_trajectory.joint_names = left_arm_joints

        right_arm_trajectory = JointTrajectory()
        right_arm_trajectory.joint_names = right_arm_joints

        # 获取各关节在control_joints中的索引
        neck_indices = [self.control_joints.index(j) for j in neck_joints]
        left_arm_indices = [self.control_joints.index(j) for j in left_arm_joints]
        right_arm_indices = [self.control_joints.index(j) for j in right_arm_joints]

        # 线性插值
        for i in range(self.steps + 1):
            # 计算插值比例 (0.0到1.0)
            alpha = i / self.steps

            # 计算所有关节的插值位置
            interpolated_pos = [
                start + alpha * (target - start)
                for start, target in zip(self.current_positions, target_positions)
            ]

            # 创建轨迹点
            point = JointTrajectoryPoint()
            point.positions = interpolated_pos
            point.time_from_start = rclpy.time.Duration(seconds=self.duration * alpha).to_msg()

            # 为每个控制器创建对应的点
            neck_point = JointTrajectoryPoint()
            neck_point.positions = [interpolated_pos[i] for i in neck_indices]
            neck_point.time_from_start = point.time_from_start

            left_arm_point = JointTrajectoryPoint()
            left_arm_point.positions = [interpolated_pos[i] for i in left_arm_indices]
            left_arm_point.time_from_start = point.time_from_start

            right_arm_point = JointTrajectoryPoint()
            right_arm_point.positions = [interpolated_pos[i] for i in right_arm_indices]
            right_arm_point.time_from_start = point.time_from_start

            # 添加到各自的轨迹
            neck_trajectory.points.append(neck_point)
            left_arm_trajectory.points.append(left_arm_point)
            right_arm_trajectory.points.append(right_arm_point)

        # 发布轨迹
        self.neck_publisher.publish(neck_trajectory)
        self.left_arm_publisher.publish(left_arm_trajectory)
        self.right_arm_publisher.publish(right_arm_trajectory)

        # self.get_logger().info(
        #     f"Published trajectories:\n"
        #     f"Neck: {neck_point.positions} -> {[target_positions[i] for i in neck_indices]}\n"
        #     f"Left Arm: {left_arm_point.positions} -> {[target_positions[i] for i in left_arm_indices]}\n"
        #     f"Right Arm: {right_arm_point.positions} -> {[target_positions[i] for i in right_arm_indices]}\n"
        #     f"Duration: {self.duration} seconds"
        # )

    def shutdown_node(self):
        self.get_logger().info("Trajectory completed, shutting down node...")
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    joint_control = JointControl()
    try:
        rclpy.spin(joint_control)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            joint_control.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
