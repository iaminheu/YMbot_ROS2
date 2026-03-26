import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np
from scipy.spatial.transform import Rotation as R
from collections import deque
import matplotlib.pyplot as plt
import os
from ament_index_python.packages import get_package_share_directory
import threading
from ik_solver_pkg.robot_control.ymbot_ikf import Ymbot_ArmIK
from ik_solver_pkg.robot_control.ymbot_collisionchecker import CollisionChecker
import time
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException
import tf_transformations
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Bool, Int32MultiArray, Float32MultiArray, String
## 初始化关节名与索引的映射
NUM_JOINTS = 20
JOINT_NAMES_LIST = [ "Left_Arm_Joint1" , "Left_Arm_Joint2" , "Left_Arm_Joint3" , "Left_Arm_Joint4" ,
                     "Left_Arm_Joint5" , "Left_Arm_Joint6" , "Left_Arm_Joint7" ,
                     "Right_Arm_Joint1", "Right_Arm_Joint2", "Right_Arm_Joint3", "Right_Arm_Joint4",
                     "Right_Arm_Joint5", "Right_Arm_Joint6", "Right_Arm_Joint7",
                     "Body_Joint1"     , "Body_Joint2"     , "Body_Joint3"     , "Body_Joint4"     ,
                     "Neck_Joint1"     , "Neck_Joint2"     , ]

class KinematicState:
    def __init__(self, positions, velocities, time_stamp):
        self.positions = np.array(positions)
        self.velocities = np.array(velocities)
        self.time_stamp = time_stamp

class SmoothServo:
    def __init__(self, node, num_joints, publish_callback, get_current_state_func, collision_checker):

        self.node = node
        self.num_joints = num_joints
        self.publish_callback = publish_callback
        self.get_current_state = get_current_state_func
        self.collision_checker = collision_checker

        # 参数配置
        self.publish_period = 0.02  # 50Hz
        self.max_velocity = np.full(num_joints, 10.0)
        self.max_acceleration = np.full(num_joints, 15.0)
        self.max_expected_latency = 0.1
        self.command_timeout = 0.2

        # 状态跟踪
        self.joint_cmd_window = deque(maxlen=10)
        self.last_command_time = self.node.get_clock().now()
        self.last_target_state = self._get_initial_state()

        self.collision_triggered = False

        # 历史记录（调试用）
        self.plot = True
        self.history_time = []
        self.history_raw_position = []
        self.history_position = []
        self.history_velocity = []

        #时间
        self._last_traj_gen_ms=None


    def _get_initial_state(self):
        current_state = self.get_current_state()
        return KinematicState(
            positions=current_state.positions,
            velocities=np.zeros(self.num_joints),
            time_stamp=self.node.get_clock().now()
        )

    def update(self, target_positions):
        current_time = self.node.get_clock().now()

        # 检查指令超时
        if (current_time - self.last_command_time) > Duration(seconds=self.command_timeout):
            self.node.get_logger().warn("Command timeout, resetting window")
            self.joint_cmd_window.clear()
            self.last_target_state = self._get_initial_state()
            self.last_command_time = current_time
            return


        # 计算原始速度
        target_velocities = (target_positions - self.last_target_state.positions) / self.publish_period

        # 构建目标状态
        target_state = KinematicState(
            positions=target_positions,
            velocities=target_velocities,
            time_stamp=current_time + Duration(seconds=self.max_expected_latency)
        )

        # 应用速度约束
        self._apply_velocity_constraints(target_state)

        # 更新滑动窗口
        self._update_sliding_window(target_state, current_time)

        # 更新碰撞检测器状态（传入速度和位置）
        self.collision_checker.update_state(target_state.positions, target_state.velocities)


        # 记录历史数据
        if self.plot:
            self.history_time.append(current_time.nanoseconds * 1e-9)
            self.history_raw_position.append(target_positions)
            self.history_position.append(target_state.positions.copy())
            self.history_velocity.append(target_state.velocities.copy())

        # 生成轨迹
        if len(self.joint_cmd_window) >= 2:
            t_traj_start =time.perf_counter()
            trajectory_tuple = self._generate_trajectory(current_time)
            t_traj_end =time.perf_counter()
            self._last_traj_gen_ms = (t_traj_end -t_traj_start) * 1000.0
            self.publish_callback(trajectory_tuple)
        else:
            print("Not enough points in window to generate trajectory!")

        self.last_target_state = target_state
        self.last_command_time = current_time

    def _apply_velocity_constraints(self, target_state):
        # 首先应用碰撞方向过滤
        if self.collision_checker.is_collision():
            filtered_velocities = self.collision_checker.filter_velocity_direction(target_state.velocities)
            target_state.velocities = filtered_velocities
        
        # 然后应用速度缩放
        velocity_scale = np.minimum(
            np.abs(self.max_velocity) / (np.abs(target_state.velocities) + 1e-6),1.0
        )

        target_state.velocities *= velocity_scale
        target_state.positions = self.last_target_state.positions + target_state.velocities * self.publish_period


    def _update_sliding_window(self, new_state, current_time):
        cutoff_time = current_time - Duration(seconds=self.max_expected_latency)
        while len(self.joint_cmd_window) > 2 and self.joint_cmd_window[0].time_stamp < cutoff_time:
            self.joint_cmd_window.popleft()
        self.joint_cmd_window.append(new_state)


    def _generate_trajectory(self, current_time):
        left_traj = JointTrajectory()
        right_traj = JointTrajectory()
        left_traj.header.stamp = current_time.to_msg()
        right_traj.header.stamp = current_time.to_msg()
        left_traj.joint_names = [f"Left_Arm_Joint{i+1}" for i in range(7)]
        right_traj.joint_names = [f"Right_Arm_Joint{i+1}" for i in range(7)]

        # 方向性碰撞限制处理
        # 注意：碰撞方向过滤已经在_apply_velocity_constraints中应用
        # 这里不再需要完全停止，而是允许非碰撞方向的运动
        
        # 正常轨迹生成（碰撞方向已被过滤）
        for i in range(len(self.joint_cmd_window)-1):
            start = self.joint_cmd_window[i]
            end = self.joint_cmd_window[i+1]
            dt = (end.time_stamp - start.time_stamp).nanoseconds / 1e9

            if dt >= 0.005:
                point_left = JointTrajectoryPoint()
                point_left.positions = end.positions[:7].tolist()
                point_left.velocities = end.velocities[:7].tolist()
                point_left.time_from_start = (end.time_stamp - current_time).to_msg()

                point_right = JointTrajectoryPoint()
                point_right.positions = end.positions[7:14].tolist()
                point_right.velocities = end.velocities[7:14].tolist()
                point_right.time_from_start = (end.time_stamp - current_time).to_msg()

                left_traj.points.append(point_left)
                right_traj.points.append(point_right)

        return left_traj, right_traj


    def plot_history(self):
        if not self.plot:
            return
        """历史数据绘图（调试用）"""
        num_joints = self.num_joints - 6
        time_stamps = np.array(self.history_time)
        pos_history = np.array(self.history_position)
        vel_history = np.array(self.history_velocity)
        raw_pos_histiry = np.array(self.history_raw_position)

        fig, axes = plt.subplots(6, 5, figsize=(20, 15))
        fig.suptitle("Joint Position and Velocity History", fontsize=16)
        for i in range(num_joints):
            row = (i // 5) * 2
            col = i % 5
            ax_pos = axes[row][col]
            ax_vel = axes[row + 1][col]
            ax_pos.plot(time_stamps, pos_history[:, i])
            ax_pos.set_title(f"Joint {i} - Position")
            ax_pos.set_ylabel("rad")
            ax_pos.grid(True)
            ax_vel.plot(time_stamps, vel_history[:, i])
            ax_vel.set_title(f"Joint {i} - Velocity")
            ax_vel.set_xlabel("Time [s]")
            ax_vel.set_ylabel("rad/s")
            ax_vel.grid(True)
        # 删除多余子图（第15个位置）
        axes[-1][-1].axis("off")
        axes[-2][-1].axis("off")

        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plt.show()


class IKServoNode_Coll(Node):
    def __init__(self):
        super().__init__('ik_servo_node_coll')
        self.arm_ik = None



        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_cb,
            10
        )

        # 机器人从/joint_states 初始化状态
        self._collision_init_done = False
        self.collision_checker = None
        self.joint_positions = np.zeros(NUM_JOINTS)
        # 订阅器
        self.left_pose = None
        self.right_pose = None
        self.create_subscription(Pose, '/arm_left/ee_status', self.left_cb, 10)
        self.create_subscription(Pose, '/arm_right/ee_status', self.right_cb, 10)
        # self.create_subscription(Float32MultiArray, 'vr_body_joints_vel_cmds', self.body_cb, 1)
        # self.create_subscription(Float32MultiArray, 'vr_neck_joints_vel_cmds', self.neck_cb, 1)
        # self.create_subscription(Float32MultiArray, '/chunk_joints_vel_cmds', self.chunk_cb, 1)
        self.create_subscription(Float32MultiArray, '/torso_joints_vel', self.chunk_cb, 1)
        # 定时器
        self.solve_timer = self.create_timer(0.02, self.try_solve)

        # 新增TF相关初始化
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(String, 'reset_command', self.reset_cb, 10)
        # 新增位姿跟踪状态
        self.is_first_left_pose_received = False
        self.is_first_right_pose_received = False
        self.last_left_pose_received_time = self.get_clock().now()
        self.last_right_pose_received_time = self.get_clock().now()

        self.first_vr_pose_left = None
        self.first_robo_pose_left = None

        self.first_vr_pose_right = None
        self.first_robo_pose_right = None

        # 新增超时检测定时器
        self.pose_timeout_timer = self.create_timer(0.5, self.check_pose_timeout)


        # 发布器
        self.left_traj_pub = self.create_publisher(
            JointTrajectory,
            '/left_arm_controller/joint_trajectory',
            1
        )
        self.right_traj_pub = self.create_publisher(
            JointTrajectory,
            '/right_arm_controller/joint_trajectory',
            1
        )
        self.body_traj_pub = self.create_publisher(
            JointTrajectory,
            '/body_controller/joint_trajectory',
            1
        )
        self.neck_traj_pub = self.create_publisher(
            JointTrajectory,
            '/neck_controller/joint_trajectory',
            1
        )
        self.left_target_pub = self.create_publisher(
            JointTrajectory,
            '/left_arm/target',
            1
        )
        self.right_target_pub = self.create_publisher(
            JointTrajectory,
            '/right_arm/target',
            1
        )
        self.chunk_target_pub = self.create_publisher(
            Float32MultiArray,
            '/chunk/target',
            1
        )


        # 平滑控制器
        self.current_robot_state = None
        self.servo = None

        #复位标记
        self.reseting = False

        #可视化
        self.dispaly = False

        self._t_left_pose =None
        self._t_right_pose =None

    def reset_cb(self, msg):
        '''电机复位处理'''
        command = msg.data
        if command != 'reset':
            print("get unexpected string data")
            return
        print('reseting positions...')
        self.reseting = True
        steps = 500
        self.current_positions = self.joint_positions.copy()
        reset_positions = np.array([-0.35, 0.0, 0.0, -0.52, 0.0, 0.0, 0.0,
                                    0.35, 0.0, 0.0,  0.52, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0,   0.0,
                                    0.0, -0.59])
        # self.current_positions = reset_positions

        assert len(self.current_positions) == NUM_JOINTS

        self.reset_dx = np.zeros(NUM_JOINTS)

        for i in range(NUM_JOINTS):
            self.reset_dx[i] = (self.current_positions[i] - reset_positions[i])/steps

        for j in range(steps):
            for i in range(NUM_JOINTS):
                self.current_positions[i] -= self.reset_dx[i]
            left_arm = self.servo.last_target_state.positions[:7]
            right_arm = self.servo.last_target_state.positions[7:14]
            if self.dispaly:
                self.arm_ik.vis.display(np.concatenate([left_arm, right_arm]))
            self.servo.update(self.current_positions)
            time.sleep(0.005)

        self.arm_ik.reset(self.current_positions)
        self.get_logger().info("reset pose")
        self.servo.joint_cmd_window.clear()
        self.servo.last_target_state = self.servo._get_initial_state()
        self.left_pose = None
        self.right_pose = None
        self.is_first_left_pose_received = False
        self.first_robo_pose_left = None
        self.first_vr_pose_left = None
        self.is_first_right_pose_received = False
        self.first_robo_pose_right = None
        self.first_vr_pose_right = None
        self.reseting = False
        print('reset done')



    def check_pose_timeout(self):
        """位姿指令超时检测"""
        now = self.get_clock().now()
        if self.is_first_left_pose_received and (now - self.last_left_pose_received_time).nanoseconds > 1e9:
            self.get_logger().warn("Pose timeout! Resetting initial pose.")
            self.is_first_left_pose_received = False
            self.first_robo_pose_left = None
            self.first_vr_pose_left = None

        if self.is_first_right_pose_received and (now - self.last_right_pose_received_time).nanoseconds > 1e9:
            self.get_logger().warn("Pose timeout! Resetting initial pose.")
            self.is_first_right_pose_received = False
            self.first_robo_pose_right = None
            self.first_vr_pose_right = None

    def chunk_cb(self, msg):
        """
        处理躯干和颈部关节的速度指令，采用柔顺控制
        msg.data format: [body_joint1, body_joint2, body_joint3, body_joint4, neck_joint1, neck_joint2]
        """
        if self.reseting or not self._collision_init_done:
            self.get_logger().warn("Reseting or not init collision detect!")
            return
        
        # 分离躯干和颈部数据
        body_data = msg.data[:4]  # 前4个是躯干关节
        neck_data = msg.data[4:]  # 后2个是颈部关节
        
        # 创建柔顺控制参数
        max_velocity = 2.0  # 最大速度 (rad/s)
        max_acceleration = 2.0  # 最大加速度 (rad/s²)
        control_period = 0.02  # 控制周期 (20ms)
        
        # 获取当前位置
        current_body_positions = self.joint_positions[14:18]
        current_neck_positions = self.joint_positions[18:20]
        
        # 计算速度指令（速度模式）
        body_velocities = np.array(body_data)  # 直接作为速度指令
        neck_velocities = np.array(neck_data)
        
        # 应用速度限制
        body_velocities = np.clip(body_velocities, -max_velocity, max_velocity)
        neck_velocities = np.clip(neck_velocities, -max_velocity, max_velocity)
        
        # 计算目标位置（位置增量）
        body_target_positions = current_body_positions + body_velocities * control_period
        neck_target_positions = current_neck_positions + neck_velocities * control_period
        
        # 创建轨迹消息
        if any(body_velocities != 0):
            body_traj = JointTrajectory()
            body_traj.header.stamp = self.get_clock().now().to_msg()
            body_traj.joint_names = [f"Body_Joint{i+1}" for i in range(4)]
            
            point_body = JointTrajectoryPoint()
            point_body.positions = body_target_positions.tolist()
            point_body.velocities = body_velocities.tolist()  # 使用计算出的速度
            point_body.time_from_start = Duration(seconds=control_period).to_msg()
            body_traj.points.append(point_body)
            
            self.body_traj_pub.publish(body_traj)
        
        if any(neck_velocities != 0):
            neck_traj = JointTrajectory()
            neck_traj.header.stamp = self.get_clock().now().to_msg()
            neck_traj.joint_names = [f"Neck_Joint{i+1}" for i in range(2)]
            
            point_neck = JointTrajectoryPoint()
            point_neck.positions = neck_target_positions.tolist()
            point_neck.velocities = neck_velocities.tolist()  # 使用计算出的速度
            point_neck.time_from_start = Duration(seconds=control_period).to_msg()
            neck_traj.points.append(point_neck)
            
            self.neck_traj_pub.publish(neck_traj)
        
        # 发布目标位置（用于显示或其他用途）
        chunk_msg = Float32MultiArray()
        combined_positions = list(body_target_positions) + list(neck_target_positions)
        chunk_msg.data = combined_positions

        self.chunk_target_pub.publish(chunk_msg)

    def left_cb(self, msg):
        if self.reseting or not self._collision_init_done:
            return
        """处理左臂位姿指令"""
        self.last_left_pose_received_time = self.get_clock().now()
        self._t_left_pose=time.perf_counter()

        if not self.is_first_left_pose_received:
            # 初始化参考位姿
            self.first_vr_pose_left = msg
            try:
                # 获取当前实际末端位姿
                transform = self.tf_buffer.lookup_transform(
                    'base_link',
                    'Left_Arm_Link8',
                    rclpy.time.Time()
                )
                self.first_robo_pose_left = self.transform_to_pose(transform)
                self.is_first_left_pose_received = True
                self.get_logger().info("Initial left pose set")
            except TransformException as e:
                self.get_logger().error(f"TF lookup failed: {e}")
                return
        else:
            # 计算相对位姿增量
            relative_pose = self.calculate_relative_pose(msg, self.first_vr_pose_left)

            # 生成目标位姿
            target_pose = self.apply_relative_pose(
                self.first_robo_pose_left,
                relative_pose
            )

            self.left_pose = self.pose_to_matrix(target_pose)

    def right_cb(self, msg):
        if self.reseting or not self._collision_init_done:
            return
        """处理右臂位姿指令"""
        self.last_right_pose_received_time = self.get_clock().now()
        self._t_right_pose=time.perf_counter()

        if not self.is_first_right_pose_received:
            # 初始化参考位姿
            self.first_vr_pose_right = msg
            try:
                # 获取当前实际末端位姿
                transform = self.tf_buffer.lookup_transform(
                    'base_link',
                    'Right_Arm_Link8',
                    rclpy.time.Time()
                )
                self.first_robo_pose_right = self.transform_to_pose(transform)
                self.is_first_right_pose_received = True
                self.get_logger().info("Initial right pose set")
            except TransformException as e:
                self.get_logger().error(f"TF lookup failed: {e}")
                return
        else:
            # 计算相对位姿增量
            relative_pose = self.calculate_relative_pose(msg, self.first_vr_pose_right)

            # 生成目标位姿
            target_pose = self.apply_relative_pose(
                self.first_robo_pose_right,
                relative_pose
            )
            self.right_pose = self.pose_to_matrix(target_pose)

    def transform_to_pose(self, transform):
        """将TransformStamped转换为Pose消息"""
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose

    def calculate_relative_pose(self, current_pose, first_pose):
        """计算相对位姿增量"""
        # 位置增量
        rel_pos = [
            current_pose.position.x - first_pose.position.x,
            current_pose.position.y - first_pose.position.y,
            current_pose.position.z - first_pose.position.z
        ]

        # 从Pose消息直接提取四元数
        q_current = [
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w
        ]
        q_first = [
            first_pose.orientation.x,
            first_pose.orientation.y,
            first_pose.orientation.z,
            first_pose.orientation.w
        ]

        # 计算相对旋转
        q_rel = tf_transformations.quaternion_multiply(
            q_current,
            tf_transformations.quaternion_inverse(q_first)
        )

        # 构造相对位姿
        rel_pose = Pose()
        rel_pose.position.x = rel_pos[0]
        rel_pose.position.y = rel_pos[1]
        rel_pose.position.z = rel_pos[2]
        rel_pose.orientation.x = q_rel[0]
        rel_pose.orientation.y = q_rel[1]
        rel_pose.orientation.z = q_rel[2]
        rel_pose.orientation.w = q_rel[3]
        return rel_pose

    # 同时修改 apply_relative_pose 方法：
    def apply_relative_pose(self, base_pose, relative_pose):
        """应用相对位姿到基准位姿"""
        # 位置叠加
        target_pos = [
            base_pose.position.x + relative_pose.position.x,
            base_pose.position.y + relative_pose.position.y,
            base_pose.position.z + relative_pose.position.z
        ]

        # 从Pose消息提取四元数
        q_base = [
            base_pose.orientation.x,
            base_pose.orientation.y,
            base_pose.orientation.z,
            base_pose.orientation.w
        ]
        q_rel = [
            relative_pose.orientation.x,
            relative_pose.orientation.y,
            relative_pose.orientation.z,
            relative_pose.orientation.w
        ]

        # 计算合成旋转
        q_target = tf_transformations.quaternion_multiply(q_rel, q_base)

        # 构造目标位姿
        target_pose = Pose()
        target_pose.position.x = target_pos[0]
        target_pose.position.y = target_pos[1]
        target_pose.position.z = target_pos[2]
        target_pose.orientation.x = q_target[0]
        target_pose.orientation.y = q_target[1]
        target_pose.orientation.z = q_target[2]
        target_pose.orientation.w = q_target[3]
        return target_pose

    def joint_state_cb(self, msg):
        ## 关节索引要对应
        positions = np.zeros(NUM_JOINTS)
        velocities = np.zeros(NUM_JOINTS)
        for i, name in enumerate(JOINT_NAMES_LIST):
            idx = msg.name.index(name)
            positions[i] = msg.position[idx]
            self.joint_positions[i] = positions[i]
            velocities[i] = msg.velocity[idx]
        if not np.isnan(positions).any():
            self.current_robot_state = KinematicState(
                positions=positions,
                velocities=velocities,
                time_stamp=self.get_clock().now()
            )
            if not self._collision_init_done:
                self._init_collision_checker(positions)
                self._collision_init_done = True
                print("self._collision_init_done = True")
                self.arm_ik = Ymbot_ArmIK(init_positions = positions,Visualization=False)
                self.servo = SmoothServo(
                        node=self,
                        num_joints=20,
                        publish_callback=self.publish_joint_command,
                        get_current_state_func=self.get_current_robot_state,
                        collision_checker=self.collision_checker
                    )
                msg = Float32MultiArray()
                msg.data = self.joint_positions[14:20]
                self.chunk_target_pub.publish(msg)
            #else:
            #    # 更新碰撞检测器待检测的机器人状态
            #    self.collision_checker.update_state(positions)

    def _init_collision_checker(self,init_positions):
        # 初始化碰撞检测器
        this_dir = os.path.dirname(os.path.abspath(__file__))
        urdf_path = os.path.join(
            get_package_share_directory("ymbot_d_description"),
            "urdf",
            "ymbot_d.urdf"
        )
        srdf_path = os.path.join(
            get_package_share_directory("ymbot_d_description"),
            "srdf",
            "ymbot_d.srdf"
        )
        self.collision_checker = CollisionChecker(
                                    node=self,
                                    urdf_path=urdf_path,
                                    srdf_path=srdf_path,
                                    init_positions=init_positions,
                                    check_freq=30
                                )

    def publish_joint_command(self, trajectory_tuple):
        left_traj, right_traj = trajectory_tuple
        self.left_traj_pub.publish(left_traj)
        self.right_traj_pub.publish(right_traj)
        self.left_target_pub.publish(left_traj)
        self.right_target_pub.publish(right_traj)

    def get_current_robot_state(self):
        # 如果是None则返回失败
        if self.current_robot_state is None:
            #return None
            return KinematicState(
                positions=np.zeros(20),
                velocities=np.zeros(20),
                time_stamp=self.get_clock().now()
            )
        return self.current_robot_state


    def try_solve(self):
        if self.reseting:
            return

        # 正常IK解算流程
        if self.left_pose is not None and self.right_pose is not None :

            # try:
            t_ik_start = time.perf_counter()
            sol_q, _ = self.arm_ik.solve_ik(self.left_pose, self.right_pose)
            t_ik_end = time.perf_counter()

            full_q = np.zeros(self.collision_checker.robot.model.nq) #self.collision_checker.robot.data.q.copy()

            for i in range(NUM_JOINTS):
                full_q[i] = self.collision_checker.robot.data.q[self.collision_checker.robot.model.getJointId(JOINT_NAMES_LIST[i])-1]

            full_q[:7] = sol_q[:7]    # 左臂关节
            full_q[7:14] = sol_q[7:]   # 右臂关节

            self.servo.update( full_q)
            self._last_left_pose = self.left_pose  # 更新记录值
            self._last_right_pose = self.right_pose

            # if self._t_left_pose is not None and self._t_right_pose is not None:
            #     t_pose=max(self._t_left_pose,self._t_right_pose)
            #     traj_ms =self.servo._last_traj_gen_ms
            #     traj_part = f" | traj_gen:{traj_ms:.3f}ms" if traj_ms is not None else ""
            #     self.get_logger().info(
            #         f"timing pose_to_ik:{(t_ik_start-t_pose)*1000.0:.3f}ms | "
            #         f"ik_solve:{(t_ik_end - t_ik_start)*1000.0:.3f}ms"
            #         f"{traj_part}"
            #     )

            # 可视化
            if self.dispaly:
                left_arm = self.servo.last_target_state.positions[:7]
                right_arm = self.servo.last_target_state.positions[7:14]
                self.arm_ik.vis.display(np.concatenate([left_arm, right_arm]))


    def pose_to_matrix(self, pose_msg):
        q = pose_msg.orientation
        t = pose_msg.position
        quat = [q.x, q.y, q.z, q.w]
        rot = R.from_quat(quat).as_matrix()
        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    def destroy_node(self):
        self.get_logger().info("Shutting down collision checker...")
        self.collision_checker.shutdown()
        # self.servo.plot_history()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = IKServoNode_Coll()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()