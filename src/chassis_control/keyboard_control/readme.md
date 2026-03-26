#### 编译说明

```
1. colcon build --symlink-install --packages-select keyboard_control
```


# 一、代码整体功能（一句话总结）

这是一个 **ROS 2 机器人键盘控制节点**，作用是：

**通过电脑键盘（方向键 / WASD / 快捷键）实时控制机器人底盘前后左右运动，并支持速度调节、电机使能、自动 / 手动模式切换。**

---

# 二、代码用途（项目定位）

这是机器人的**手动遥控程序**，相当于：

**机器人的 “键盘遥控器”**

用途包括：

1. 用键盘控制机器人前后左右移动
2. 调节最大速度
3. 开启 / 关闭机器人电机
4. 切换手动控制 / 自动导航模式
5. 安全保护：无操作时自动停止

---

# 三、核心技术说明

* 运行环境：**ROS 2 (Humble/Iron/Rolling)**
* 语言：**C++**
* 通信方式：**话题发布速度指令 Twist**
* 控制方式：**终端键盘监听（非阻塞式）**
* 多线程：**键盘监听线程 + 速度发布线程**（保证不卡顿）

---

# 四、代码结构解释（超级清晰）

## 1. 头文件部分

包含系统库 + ROS 2 核心库

cpp

运行

```
#include <fcntl.h>
#include <termios.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
```

作用：

* `termios.h`：监听键盘按键（方向键、WASD）
* `twist.hpp`：ROS 2 速度消息（前后左右）
* `rclcpp`：ROS 2 C++ 核心 API

---

## 2. 宏定义（快捷键 + 速度限制）

cpp

运行

```
#define KEYCODE_RIGHT_ARROW 0x43
#define KEYCODE_LEFT_ARROW 0x44
#define KEYCODE_UP_ARROW 0x41
#define KEYCODE_DOWN_ARROW 0x42

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
...

#define LINEAR_MAX 0.5
#define ANGULAR_MAX 0.5
```

作用：

* 定义键盘按键编码（方向键、WASD、QZEC 等）
* 定义机器人最大速度、最小速度
* 定义发布频率（20Hz）

---

# 五、核心类：Keyboard 键盘控制类

cpp

运行

```
class Keyboard : public rclcpp::Node
```

## 成员变量作用

* `linear_`：前后速度
* `angular_`：转向速度
* `linear_scale_`：线性速度比例
* `angular_scale_`：转向速度比例
* `motion_status_`：电机是否使能
* `auto_mode_`：是否自动模式
* `vel_publisher_`：发布速度话题 `/key_cmd_vel`
* `mortor_enable_publisher_`：发布电机使能 `/en_chassis_motor`

---

# 六、构造函数：初始化参数 + 创建发布者

cpp

运行

```
Keyboard::Keyboard()
```

作用：

1. 读取配置参数（速度、最大速度、发布频率等）
2. 若无配置，则使用默认值
3. 创建两个发布者：
   * `/key_cmd_vel`：速度控制
   * `/en_chassis_motor`：电机开关

---

# 七、Loop ()：键盘监听主函数（最重要）

cpp

运行

```
void Keyboard::Loop()
```

## 功能：

1. **设置终端为原始模式**，可以直接读取方向键
2. **循环监听键盘按键**
3. **根据按键执行不同动作**

---

# 八、按键功能说明（你最关心的）

## 1. 方向控制

* **↑ / W**：前进
* **↓ / S**：后退
* **← / A**：左转
* **→ / D**：右转

## 2. 速度调节

* **Q**：增加直线速度
* **Z**：减少直线速度
* **E**：增加转向速度
* **C**：减少转向速度

## 3. 电机控制

* **B**：电机使能（ENABLE）
* **N**：电机禁用（DISABLE）

## 4. 模式切换

* **O**：自动模式（Auto）
* **P**：手动键盘模式
* **空格**：急停

## 5. 退出

* **CTRL + C**

---

# 九、PubThread ()：发布速度线程（独立线程）

cpp

运行

```
void Keyboard::PubThread()
```

作用：

* 以 **20Hz 固定频率** 发布速度
* **无操作 0.1 秒 自动停止**（安全机制）
* 自动模式下不发布速度（交给导航）

---

# 十、main () 主函数

cpp

运行

```
int main()
```

作用：

1. 初始化 ROS 2
2. 注册 `CTRL+C` 退出信号
3. **创建两个线程**：
   * 线程 1：Loop () 键盘监听
   * 线程 2：PubThread () 速度发布
4. 保持节点运行

---

# 十一、程序运行流程（最清晰总结）

1. 启动程序
2. 终端提示按键说明
3. 监听键盘
4. 按键 → 计算速度
5. 发布速度话题 `/key_cmd_vel`
6. 机器人底盘接收并运动
7. 松开按键 → 自动停止
8. 按 B 启动电机 / N 关闭电机
9. 按 O 自动 / P 手动
10. CTRL+C 退出，恢复终端

---

# 十二、该代码在机器人系统中的地位

这是机器人的**手动遥控核心节点**，几乎所有轮式机器人都会用。

它的输出：

* `/key_cmd_vel` 速度指令
* `/en_chassis_motor` 电机使能

给：

* 底盘驱动
* 运动控制器
* 仿真环境

---

# 十三、最简总结（你可以直接写进报告）

这是一个 **ROS 2 机器人键盘遥控程序**，通过监听键盘事件，实时发布机器人速度指令，

支持方向控制、速度调节、电机使能、自动 / 手动模式切换，

并具备无操作自动停止的安全机制，用于机器人手动控制、调试与测试。
