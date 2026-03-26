import casadi                                                                       
import meshcat.geometry as mg
import numpy as np
import pinocchio as pin                             
import time
from pinocchio import casadi as cpin                
from pinocchio.robot_wrapper import RobotWrapper    
from pinocchio.visualize import MeshcatVisualizer   
import os
import sys
from ament_index_python.packages import get_package_share_directory
import time


## 初始化关节名与索引的映射
JOINT_NAMES_LIST = [ "Left_Arm_Joint1", "Left_Arm_Joint2", "Left_Arm_Joint3", "Left_Arm_Joint4", 
                    "Left_Arm_Joint5", "Left_Arm_Joint6", "Left_Arm_Joint7", 
                    "Right_Arm_Joint1", "Right_Arm_Joint2", "Right_Arm_Joint3", "Right_Arm_Joint4", 
                    "Right_Arm_Joint5", "Right_Arm_Joint6", "Right_Arm_Joint7",
                    "Body_Joint1", "Body_Joint2", "Body_Joint3", "Body_Joint4", 
                    "Neck_Joint1", "Neck_Joint2", ]

NUM_JOINTS = 20

from ik_solver_pkg.utils.weighted_moving_filter import WeightedMovingFilter

class Ymbot_ArmIK:
    def __init__(self, init_positions , Unit_Test = False, Visualization = False):
        np.set_printoptions(precision=5, suppress=True, linewidth=200)

        self.Unit_Test = Unit_Test
        self.Visualization = Visualization

        # 转成规范路径（去掉多余的 ../）
        urdf_path = os.path.join(
            get_package_share_directory("ymbot_d_description"),
            "urdf",
            "ymbot_d.urdf"
        )
        mesh_dir = os.path.join(
            get_package_share_directory("ymbot_d_description"),
            "meshes",
        )

        self.robot = pin.RobotWrapper.BuildFromURDF(urdf_path, [mesh_dir])

         # 设置特定关节的角度
        self.set_initial_joint_angles(init_positions)
        self.mixed_jointsToLockIDs = [
                                        "Body_Joint1",
                                        "Body_Joint2",
                                        "Body_Joint3",
                                        "Body_Joint4",
                                        "Neck_Joint1",
                                        "Neck_Joint2"                         
                                     ]

        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=self.robot.data.q,
        )


        self.reduced_robot.data = self.reduced_robot.model.createData()
        # initial_q = pin.neutral(self.reduced_robot.model)  # 获取中立位置的关节角度
        # #设置手臂初始位置

        self.reduced_robot.model.addFrame(
            pin.Frame('L_ee',
                      self.reduced_robot.model.getJointId('Left_Arm_Joint7'),
                      pin.SE3(np.eye(3),
                              np.array([0,0,-0.1267]).T),
                      pin.FrameType.OP_FRAME)
        )
        
        self.reduced_robot.model.addFrame(
            pin.Frame('R_ee',
                      self.reduced_robot.model.getJointId('Right_Arm_Joint7'),
                      pin.SE3(np.eye(3),
                              np.array([0,0,-0.1267]).T),
                      pin.FrameType.OP_FRAME)
        )
        self.reduced_robot.data.q = init_positions[:14]

        # for i in range(self.reduced_robot.model.nframes):
        #     frame = self.reduced_robot.model.frames[i]
        #     frame_id = self.reduced_robot.model.getFrameId(frame.name)
        #     print(f"Frame ID: {frame_id}, Name: {frame.name}")
        
        # Creating Casadi models and data for symbolic computing
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()

        # Creating symbolic variables
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1) 
        self.cTf_l = casadi.SX.sym("tf_l", 4, 4)
        self.cTf_r = casadi.SX.sym("tf_r", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        # Get the hand joint ID and define the error function
        self.L_hand_id = self.reduced_robot.model.getFrameId("L_ee")
        self.R_hand_id = self.reduced_robot.model.getFrameId("R_ee")

        self.translational_error = casadi.Function(
            "translational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    self.cdata.oMf[self.L_hand_id].translation - self.cTf_l[:3,3],
                    self.cdata.oMf[self.R_hand_id].translation - self.cTf_r[:3,3]
                )
            ],
        )
        self.rotational_error = casadi.Function(
            "rotational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    cpin.log3(self.cdata.oMf[self.L_hand_id].rotation @ self.cTf_l[:3,:3].T),
                    cpin.log3(self.cdata.oMf[self.R_hand_id].rotation @ self.cTf_r[:3,:3].T)
                )
            ],
        )

        # Defining the optimization problem
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq)   # for smooth
        self.param_tf_l = self.opti.parameter(4, 4)
        self.param_tf_r = self.opti.parameter(4, 4)
        self.translational_cost = casadi.sumsqr(self.translational_error(self.var_q, self.param_tf_l, self.param_tf_r))
        self.rotation_cost = casadi.sumsqr(self.rotational_error(self.var_q, self.param_tf_l, self.param_tf_r))
        self.regularization_cost = casadi.sumsqr(self.var_q)
        # self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)
        smooth_delta = self.var_q - self.var_q_last
        smooth_delta_scale = 0.08

        self.smooth_cost = casadi.sum1(
            smooth_delta_scale ** 2
            *(casadi.sqrt(1+(smooth_delta/smooth_delta_scale) ** 2) -1 )
        )

        # Setting optimization constraints and goals
        self.opti.subject_to(self.opti.bounded(
            self.reduced_robot.model.lowerPositionLimit,
            self.var_q,
            self.reduced_robot.model.upperPositionLimit)
        )
        self.opti.minimize(50 * self.translational_cost + self.rotation_cost + 0.02 * self.regularization_cost + 10.0 * self.smooth_cost)
        # self.opti.minimize(50 * self.translational_cost + self.rotation_cost + 0.02 * self.regularization_cost + 5 * self.smooth_cost)

        opts = {
            # CasADi-level options
            'expand': True,
            'detect_simple_bounds': True,
            'calc_lam_p': False,  # https://github.com/casadi/casadi/wiki/FAQ:-Why-am-I-getting-%22NaN-detected%22in-my-optimization%3F
            'print_time': False,  # print or not
            # IPOPT solver options
            'ipopt.sb': 'yes',
            'ipopt.print_level': 0,
            'ipopt.max_iter': 50,
            'ipopt.tol': 1e-6,
            'ipopt.acceptable_tol': 5e-4,
            'ipopt.acceptable_iter': 5,
            'ipopt.warm_start_init_point': 'yes',
            'ipopt.derivative_test': 'none',
            'ipopt.jacobian_approximation': 'exact',
        }
        self.opti.solver("ipopt", opts)

        self.init_data = np.zeros(self.reduced_robot.model.nq)
        self.smooth_filter = WeightedMovingFilter(np.array([0.5, 0.2, 0.2, 0.1]), 14)
        # self.smooth_filter = WeightedMovingFilter(np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]), 14)
        # self.smooth_filter = WeightedMovingFilter(np.array([0.5, 0.3, 0.2]), 14)

        self.vis = None

        if self.Visualization:
            # Initialize the Meshcat visualizer for visualization
            self.vis = MeshcatVisualizer(self.reduced_robot.model, self.reduced_robot.collision_model, self.reduced_robot.visual_model)
            self.vis.initViewer(open=True) 
            self.vis.loadViewerModel("pinocchio") 
            self.vis.displayFrames(True, frame_ids=[73, 74], axis_length = 0.15, axis_width = 5)
            self.vis.display(pin.neutral(self.reduced_robot.model))

            # Enable the display of end effector target frames with short axis lengths and greater width.
            frame_viz_names = ['L_ee_target', 'R_ee_target']
            FRAME_AXIS_POSITIONS = (
                np.array([[0, 0, 0], [1, 0, 0],
                          [0, 0, 0], [0, 1, 0],
                          [0, 0, 0], [0, 0, 1]]).astype(np.float32).T
            )
            FRAME_AXIS_COLORS = (
                np.array([[1, 0, 0], [1, 0.6, 0],
                          [0, 1, 0], [0.6, 1, 0],
                          [0, 0, 1], [0, 0.6, 1]]).astype(np.float32).T
            )
            axis_length = 0.1
            axis_width = 10
            for frame_viz_name in frame_viz_names:
                self.vis.viewer[frame_viz_name].set_object(
                    mg.LineSegments(
                        mg.PointsGeometry(
                            position=axis_length * FRAME_AXIS_POSITIONS,
                            color=FRAME_AXIS_COLORS,
                        ),
                        mg.LineBasicMaterial(
                            linewidth=axis_width,
                            vertexColors=True,
                        ),
                    )
                )


    def reset(self, init_positions):
        """
        重置求解器的初始关节位置
        
        参数:
            init_positions: 外部获取的当前关节位置（长度20的列表或数组，包含所有关节角度）
        """
        print('reset ik solver')

        # 1. 检查输入有效性
        if len(init_positions) != NUM_JOINTS:
            raise ValueError(f"Expected {NUM_JOINTS} joint positions, got {len(init_positions)}")
        
        # 2. 更新完整机器人模型的关节状态
        full_q = np.zeros(self.robot.model.nq)
        for i in range(NUM_JOINTS):
            joint_id = self.robot.model.getJointId(JOINT_NAMES_LIST[i]) - 1
            full_q[joint_id] = init_positions[i]
        self.robot.data.q = full_q
    
        # 3. 更新简化机器人模型的关节状态（仅双臂）
        reduced_q = init_positions[:14]  # 前14个是双臂关节
        self.reduced_robot.data.q = reduced_q
        
        # 4. 重置Casadi求解器的初始猜测
        self.init_data = reduced_q.copy()
        
        # 5. 重置平滑滤波器状态
        # self.smooth_filter = WeightedMovingFilter(np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]), 14)
        self.smooth_filter = WeightedMovingFilter(np.array([ 0.5, 0.2, 0.2, 0.1]), 14)
        # self.smooth_filter = WeightedMovingFilter(np.array([0.5, 0.3, 0.2]), 14)

        
    def set_initial_joint_angles(self, init_positions):
        # 设置初始关节角度
        # 映射到完整模型 (根据实际关节索引调整)
        full_q = np.zeros(self.robot.model.nq)
        for i in range(NUM_JOINTS):
            full_q[self.robot.model.getJointId(JOINT_NAMES_LIST[i])-1] = init_positions[i]
            print("id: ",self.robot.model.getJointId(JOINT_NAMES_LIST[i])," JOINT_NAMES_LIST[i]:",JOINT_NAMES_LIST[i])
            print(init_positions[i])


        # 更新机器人状态
        self.robot.data.q = full_q

        # 更新机器人的数据
        #self.robot.data.q = initial_configuration       
        # 打印调试信息
        # print("Initial configuration:", initial_configuration)
        # print("Robot data.q:", self.robot.data.q)
    # If the robot arm is not the same size as your arm :)
    
    def scale_arms(self, human_left_pose, human_right_pose, human_arm_length=0.6, robot_arm_length=0.6):
        scale_factor = robot_arm_length / human_arm_length
        robot_left_pose = human_left_pose.copy()
        robot_right_pose = human_right_pose.copy()
        robot_left_pose[:3, 3] *= scale_factor
        robot_right_pose[:3, 3] *= scale_factor
        return robot_left_pose, robot_right_pose
    # 后来添加的内容
    def axis_tr(self,left_wrist,right_wrist):
        T = np.array([
            [0,0,-1,0],
            [-1,0,0,0],
            [0,1,0,0],
            [0,0,0,1]
        ])
        #left_wrist=left_wrist@np.linalg.inv(T)
        #right_wrist=right_wrist@np.linalg.inv(T)
        left_wrist=left_wrist@T
        right_wrist=right_wrist@T
        return left_wrist,right_wrist


    def solve_ik(self, left_wrist, right_wrist, current_lr_arm_motor_q = None, current_lr_arm_motor_dq = None):
        if current_lr_arm_motor_q is not None:
            self.init_data = current_lr_arm_motor_q
        self.opti.set_initial(self.var_q, self.init_data)
        left_wrist, right_wrist=self.axis_tr(left_wrist, right_wrist)  #新加的
        left_wrist, right_wrist = self.scale_arms(left_wrist, right_wrist)
        if self.Visualization:
            self.vis.viewer['L_ee_target'].set_transform(left_wrist)   # for visualization
            self.vis.viewer['R_ee_target'].set_transform(right_wrist)  # for visualization

        self.opti.set_value(self.param_tf_l, left_wrist)
        self.opti.set_value(self.param_tf_r, right_wrist)
        self.opti.set_value(self.var_q_last, self.init_data) # for smooth

        #try:
        sol = self.opti.solve()
        # sol = self.opti.solve_limited()

        sol_q = self.opti.value(self.var_q)
        self.smooth_filter.add_data(sol_q)
        sol_q = self.smooth_filter.filtered_data

        if current_lr_arm_motor_dq is not None:
            v = current_lr_arm_motor_dq * 0.0
        else:
            v = (sol_q - self.init_data) * 0.0

        self.init_data = sol_q

        sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v, np.zeros(self.reduced_robot.model.nv))

        return sol_q, sol_tauff

