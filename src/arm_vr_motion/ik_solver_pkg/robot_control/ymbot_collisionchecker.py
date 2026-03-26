import threading
import numpy as np
import pinocchio as pin
from collections import deque
import time
import os
import sys
import xml.etree.ElementTree as ET
from itertools import combinations


## 初始化关节名与索引的映射
JOINT_NAMES_LIST = [ "Left_Arm_Joint1", "Left_Arm_Joint2", "Left_Arm_Joint3", "Left_Arm_Joint4", 
                    "Left_Arm_Joint5", "Left_Arm_Joint6", "Left_Arm_Joint7", 
                    "Right_Arm_Joint1", "Right_Arm_Joint2", "Right_Arm_Joint3", "Right_Arm_Joint4", 
                    "Right_Arm_Joint5", "Right_Arm_Joint6", "Right_Arm_Joint7",
                    "Body_Joint1", "Body_Joint2", "Body_Joint3", "Body_Joint4", 
                    "Neck_Joint1", "Neck_Joint2", ]

NUM_JOINTS = 20

# 机器人实时自碰撞检测模块
class CollisionChecker:
    def __init__(self, node,  urdf_path, srdf_path, init_positions, check_freq=10, collision_recovery_steps=5):

        self.node = node
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._collision_flag = False
        self._recovery_counter = 0
        self._collision_recovery_steps = collision_recovery_steps
        self._check_interval = 1.0 / check_freq
        self._last_safe_q = None
        self._last_checked_q = None
        self._last_checked_qdot = None  # 记录上次检查时的速度
        self._collision_velocity_direction = None  # 记录导致碰撞的速度方向
        self.stop_q = None
        
        # 初始化机器人模型
        self._init_robot_model(urdf_path, srdf_path)

         # # 设置初始关节角度
        self._set_initial_joints(init_positions)
        
        # 启动检测线程
        self._thread = threading.Thread(target=self._run_detection, daemon=True)
        self._thread.start()

    def _init_robot_model(self, urdf_path, srdf_path):
        """完整机器人模型初始化"""
        # 构建URDF模型
        self.robot = pin.RobotWrapper.BuildFromURDF(
            urdf_path, 
            [os.path.dirname(urdf_path)]
        )
        
        # 构建碰撞模型
        self.robot.collision_model = pin.buildGeomFromUrdf(
            self.robot.model, 
            urdf_path, 
            pin.GeometryType.COLLISION,
            package_dirs=[os.path.dirname(urdf_path)]
        )
        
        # 配置碰撞对
        exclude_links = ['base_footprint_link', 'camera_link', "Neck_Joint1", "Neck_Joint2"]  # 根据实际模型调整
        self._setup_collision_pairs(urdf_path, srdf_path, exclude_links)
        self.robot.collision_data = pin.GeometryData(self.robot.collision_model)

    def _set_initial_joints(self,init_positions):
        """设置初始关节位置"""
        # 映射到完整模型 (根据实际关节索引调整)
        full_q = np.zeros(self.robot.model.nq)
        for i in range(NUM_JOINTS):
            full_q[self.robot.model.getJointId(JOINT_NAMES_LIST[i])-1] = init_positions[i]
            print("JOINT_NAMES_LIST[i]:",JOINT_NAMES_LIST[i])
            print("id: ",self.robot.model.getJointId(JOINT_NAMES_LIST[i]))

        # 更新机器人状态
        self.robot.data.q = full_q

    def _setup_collision_pairs(self, urdf_file, srdf_file, exclude_links):
        """配置碰撞检测对"""
        # 1. 获取所有几何对象处理后的名称
        processed_names = []
        for geom in self.robot.collision_model.geometryObjects:
            name = geom.name
            if name.endswith("_0"):
                name = name[:-2]
            processed_names.append(name)
        
        # 2. 生成几何对象ID映射表
        self.geometry_id_map = {name: idx for idx, name in enumerate(processed_names)}
        
        # 3. 从URDF获取所有可能的碰撞对
        possible_pairs = self._get_possible_pairs(urdf_file, exclude_links)
        
        # 4. 从SRDF获取已禁用的碰撞对
        disabled_pairs = self._get_disabled_pairs(srdf_file)
        
        # 5. 添加有效的碰撞对
        for link1, link2 in possible_pairs:
            if (link1, link2) not in disabled_pairs and (link2, link1) not in disabled_pairs:
                if link1 in self.geometry_id_map and link2 in self.geometry_id_map:
                    self.robot.collision_model.addCollisionPair(
                        pin.CollisionPair(self.geometry_id_map[link1], self.geometry_id_map[link2])
                    )

    def _get_possible_pairs(self, urdf_file, exclude_links):
        """从URDF获取所有可能的链接组合"""
        tree = ET.parse(urdf_file)
        root = tree.getroot()
        links = [link.attrib['name'] for link in root.findall('link') 
                if link.attrib['name'] not in exclude_links]
        return list(combinations(links, 2))

    def _get_disabled_pairs(self, srdf_file):
        """从SRDF获取已禁用的碰撞对"""
        tree = ET.parse(srdf_file)
        root = tree.getroot()
        disabled = set()
        for pair in root.findall('disable_collisions'):
            disabled.add((pair.attrib['link1'], pair.attrib['link2']))
        return disabled
       

    def update_state(self, q, qdot=None):
        """线程安全的状态更新
        Args:
            q: 关节位置
            qdot: 关节速度（可选，用于方向性碰撞检测）
        """
        with self._lock:
            # 计算速度方向（如果提供了速度，则使用；否则从位置变化计算）
            if qdot is not None:
                self._last_checked_qdot = np.array(qdot, dtype=np.float64)
            elif self._last_checked_q is not None:
                # 从位置变化估算速度方向（归一化）
                delta_q = np.array(q, dtype=np.float64) - self._last_checked_q
                norm = np.linalg.norm(delta_q)
                if norm > 1e-6:
                    self._last_checked_qdot = delta_q / norm
                else:
                    self._last_checked_qdot = None
            else:
                self._last_checked_qdot = None
            
            self._last_checked_q = np.array(q, dtype=np.float64)
            if not self._collision_flag or self._last_safe_q is None:
                self._last_safe_q = np.array(q, dtype=np.float64)

    def is_collision(self):
        """获取当前碰撞状态"""
        with self._lock:
            return self._collision_flag

    def get_safe_position(self):
        """获取最后一个安全位置"""
        with self._lock:
            return self._last_safe_q.copy() if self._last_safe_q is not None else None

    def filter_velocity_direction(self, target_velocity):
        """过滤掉朝碰撞方向的速度分量
        在碰撞方向的±45度范围内完全阻止运动
        
        Args:
            target_velocity: 目标速度向量
        Returns:
            过滤后的速度向量（如果在±45度范围内，返回零向量）
        """
        with self._lock:
            if not self._collision_flag:
                return target_velocity
            
            target_velocity = np.array(target_velocity, dtype=np.float64)
            
            # 如果没有速度方向信息（静止时检测到碰撞），阻止所有运动
            if self._collision_velocity_direction is None:
                self.node.get_logger().warn(
                    "Collision detected with zero velocity. Blocking all movement."
                )
                return np.zeros_like(target_velocity)
            
            # 计算目标速度的模长
            target_velocity_norm = np.linalg.norm(target_velocity)
            
            # 如果速度为零，直接返回
            if target_velocity_norm < 1e-6:
                return target_velocity
            
            # 计算目标速度在碰撞方向上的投影
            collision_dir = self._collision_velocity_direction
            dot_product = np.dot(target_velocity, collision_dir)
            
            # 计算归一化的点积（用于计算夹角）
            # cos(θ) = dot_product / (||v|| * ||d||) = dot_product / ||v||（因为d是归一化的）
            normalized_dot = dot_product / target_velocity_norm
            
            # cos(45°) ≈ 0.707
            # 如果 |cos(θ)| > cos(45°)，说明夹角 < 45度，需要完全阻止
            cos_45_deg = np.cos(np.deg2rad(45))  # ≈ 0.707
            
            if abs(normalized_dot) > cos_45_deg:
                # 夹角在±45度范围内，完全阻止运动
                angle_deg = np.degrees(np.arccos(np.clip(abs(normalized_dot), 0, 1)))
                self.node.get_logger().debug(
                    f"Blocking movement: angle={angle_deg:.1f}° < 45°, "
                    f"normalized_dot={normalized_dot:.4f}, "
                    f"velocity_norm={target_velocity_norm:.4f}"
                )
                return np.zeros_like(target_velocity)
            else:
                # 夹角大于45度，允许运动
                angle_deg = np.degrees(np.arccos(np.clip(abs(normalized_dot), 0, 1)))
                self.node.get_logger().debug(
                    f"Allowing movement: angle={angle_deg:.1f}° >= 45°, "
                    f"normalized_dot={normalized_dot:.4f}"
                )
                return target_velocity

    def shutdown(self):
        """安全停止线程"""
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
            if self._thread.is_alive():
                print("Warning: Collision thread did not shutdown cleanly")

    def _run_detection(self):
        """检测线程主循环"""
        while not self._stop_event.is_set():
            start_time = time.time()
            
            # 获取待检测状态
            with self._lock:
                q_check = self._last_checked_q
            
            if q_check is not None:
                collision = self._perform_collision_check(q_check)
                self._update_collision_state(collision, q_check)
            
            # 精确频率控制
            elapsed = time.time() - start_time
            time.sleep(max(0, self._check_interval - elapsed))

    def _perform_collision_check(self, q):
        """执行实际碰撞检测"""
        # 映射到完整模型 (根据实际关节索引调整)
        full_q = self.robot.data.q.copy()
        for i in range(NUM_JOINTS):
            full_q[self.robot.model.getJointId(JOINT_NAMES_LIST[i])-1] = q[i]
        
        # 更新机器人状态
        self.robot.data.q = full_q
        pin.forwardKinematics(self.robot.model, self.robot.data, full_q)
        
        # 更新几何位置
        pin.updateGeometryPlacements(
            self.robot.model, self.robot.data,
            self.robot.collision_model, self.robot.collision_data,
            full_q
        )
        
        # 执行碰撞检测
        return pin.computeCollisions(
            self.robot.collision_model, 
            self.robot.collision_data, 
            False  # 不停止在第一个碰撞
        )
    
    def _update_collision_state(self, collision_detected, current_q):
        """更新碰撞状态机（方向性限制模式）"""
        with self._lock:
            if collision_detected:
                # 如果是新检测到的碰撞，记录导致碰撞的速度方向
                if not self._collision_flag and self._last_checked_qdot is not None:
                    # 归一化速度方向
                    norm = np.linalg.norm(self._last_checked_qdot)
                    if norm > 1e-6:
                        self._collision_velocity_direction = self._last_checked_qdot / norm
                        self.stop_q = current_q.copy()
                        self.node.get_logger().warn(
                            f"COLLISION DETECTED! Blocking movement in direction. "
                            f"Direction norm: {norm:.4f}"
                        )
                    else:
                        # 如果没有速度信息，设置为零向量（阻止所有运动）
                        self._collision_velocity_direction = np.zeros_like(current_q)
                        self.node.get_logger().warn(
                            "COLLISION DETECTED! No velocity direction available, blocking all movement."
                        )
                
                self._collision_flag = True
                
                # 打印所有碰撞对的碰撞状态
                for k in range(len(self.robot.collision_model.collisionPairs)):
                    cr = self.robot.collision_data.collisionResults[k]
                    cp = self.robot.collision_model.collisionPairs[k]
                    geom1 = self.robot.collision_model.geometryObjects[cp.first]
                    geom2 = self.robot.collision_model.geometryObjects[cp.second]
                    if cr.isCollision():
                        self.node.get_logger().warn(f"Collision pair: {geom1.name} & {geom2.name}")

            else:
                # 不再碰撞，清除碰撞标志和方向限制
                if self._collision_flag:
                    self.node.get_logger().info("Collision cleared! Movement restrictions removed.")
                
                self._collision_flag = False
                self._collision_velocity_direction = None
                self._last_safe_q = current_q.copy()  # 更新安全位置

