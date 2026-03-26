#### 编译说明

```
1. conda activate pin
2. colcon build --symlink-install --packages-select ik_solver_pkg
```



# arm\_vr\_motion 项目分析文档

**版本：v1.0 | 日期：2026-03-26 | 格式：标准 Markdown**

---

## 一、项目核心用途与定位

本项目是一个基于 **ROS（Robot Operating System）** 开发的 **VR 沉浸式机械臂运动控制系统**。

核心目的：

**通过 VR 设备采集人体动作指令 → 经过 IK 逆运动学求解 → 控制机械臂完成高精度运动。**

---

## 二、项目目录结构

表格


| 路径             | 类型 | 功能                       |
| ---------------- | ---- | -------------------------- |
| package.xml      | 配置 | ROS 包定义，依赖，版本     |
| setup.py         | 构建 | Python 模块安装            |
| launch/          | 启动 | 一键启动 VR/IK/ 机械臂节点 |
| ik\_solver\_pkg/ | 核心 | 逆运动学求解器             |
| readme.md        | 文档 | 说明，部署，运行命令       |

---

## 三、项目整体运行流程（完整详细版）

### 阶段 1：启动（roslaunch）

执行：

`roslaunch arm_vr_motion main.launch`

自动启动：

* VR 驱动节点（采集位置、姿态、按键）
* IK 求解节点（核心计算）
* 机械臂驱动 / 仿真节点（真实臂或 Gazebo）
* 安全校验、坐标转换、滤波节点

---

### 阶段 2：VR 指令采集

* VR 手柄输出 **6DoF 位姿**（3 位置 + 3 姿态）
* 通过 ROS 话题发布：
  * `/vr/controller/pose`
  * `/vr/button/cmd`
* 进行滤波、去抖动、坐标对齐

---

### 阶段 3：IK 逆运动学求解（核心）

**输入：** 末端目标位姿 + 机械臂模型

**处理：**

* 解析解 / 数值解求解关节角
* 避奇异点、避限位
* 选最优路径（平滑、短）
  **输出：** 关节角度指令（JointState）

---

### 阶段 4：机械臂执行

* 关节指令发送至控制器
* 机械臂按轨迹运动
* 反馈当前状态（关节、末端位姿）
* VR 实时看到反馈（闭环控制）

---

### 阶段 5：安全与异常处理

* VR 急停 → 立即停止
* 目标不可达 → 报错并停止
* 碰撞 / 超限 → 强制停机
* 支持回零、暂停、续跑

---

## 四、关键技术要点

* **ROS 节点化架构**（解耦、易扩展）
* **VR 沉浸式交互**（自然、直观）
* **IK 求解实时性**（10Hz+ 控制流畅）
* **坐标系精准标定**（VR 与机械臂空间对齐）
* **多级安全保护**（急停、限位、避障）


## 五、总结

本项目完整流程为：

**VR 操作 → 数据采集 → 坐标变换 → IK 求解 → 机械臂运动 → 实时反馈 → 安全防护**

是一套可落地、可量产、可扩展的 **VR 控制机械臂解决方案**。


# 代码运行全流程（逐步骤）

## 步骤 1：启动初始化

* 订阅 `/joint_states` → 获取真实机器人关节角度
* 初始化 **碰撞检测器**
* 初始化 **IK 求解器**
* 初始化 **SmoothServo 平滑控制器**

## 步骤 2：第一次 VR 数据对齐（关键）

第一次收到 VR 位姿时：

* 记录当前 **机器人实际末端位姿 (TF)**
* 记录当前 **VR 手柄位姿**
  → **建立相对坐标系**
  之后所有运动 = **手柄相对运动 = 机器人相对运动**

## 步骤 3：VR 数据 → 机器人目标位姿

每次收到 VR 位姿：

* 计算 **手柄相对位移**
* 叠加到 **机器人初始位姿**
* 得到 **机器人目标位姿**

## 步骤 4：IK 逆解

输入：**左手目标位姿 + 右手目标位姿**

输出：**左右臂各 7 个关节角度**

## 步骤 5：SmoothServo 平滑运动（核心）

* 限制速度
* 限制加速度
* **碰撞检测：危险方向速度清零**
* 生成平滑轨迹
* 50Hz 发布

## 步骤 6：发布轨迹给电机

* `/left_arm_controller/joint_trajectory`
* `/right_arm_controller/joint_trajectory`
* `/body_controller/joint_trajectory`
* `/neck_controller/joint_trajectory`

# 🛡️ 安全机制（代码里的保护）

## 1. 碰撞保护

plaintext

```
collision_checker.filter_velocity_direction()
```

* 检测到自碰撞风险
* **把碰撞方向的速度清零**
* 非碰撞方向可继续运动
  → **不会伤机器人、不会卡电机**

## 2. 指令超时保护

超过 1 秒没收到 VR 数据 → **自动解锁重对齐**

## 3. 速度 / 加速度硬限

plaintext

```
max_velocity = 10.0
max_acceleration = 15.0
```

防止电机飞车、冲击

## 4. 复位机制（reset）

收到 `reset_command` → 机器人自动回到安全姿势

---

# 🎯 躯干 / 颈部控制（柔顺控制）

plaintext

```
chunk_cb(/torso_joints_vel)
```

* 接收 **速度指令**
* 直接柔顺控制
* 不经过 IK
* 独立发布轨迹

---

# 📡 订阅与发布清单

## 订阅（输入）

* `/joint_states` → 真实关节角度
* `/arm_left/ee_status` → 左臂 VR 位姿
* `/arm_right/ee_status` → 右臂 VR 位姿
* `/torso_joints_vel` → 躯干 + 颈部速度
* `reset_command` → 复位指令

## 发布（输出）

* `/left_arm_controller/joint_trajectory`
* `/right_arm_controller/joint_trajectory`
* `/body_controller/joint_trajectory`
* `/neck_controller/joint_trajectory`

---

# ⚡ 性能与频率

* IK 求解频率：**50Hz**
* SmoothServo 输出频率：**50Hz**
* 碰撞检测：**30Hz**
* TF 监听：实时
* 控制周期：**20ms**

---

# ✅ 最核心的 3 个亮点

1. **相对位姿控制**

   第一次对齐后，手柄怎么动，机器人就怎么动，自然不飘。
2. **SmoothServo 运动平滑**

   运动丝滑、无抖动、无冲击、保护电机。
3. **实时碰撞规避**

   机器人不会自己撞自己，安全可靠。

---

# 🎯 最终一句话总结

**这是 YMbot 机器人的 “双臂 + 躯干 + 颈部总控大脑”，

负责把 VR 动作翻译成机器人能执行的安全、平滑、精准的运动。**
