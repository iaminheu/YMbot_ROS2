# 一、一句话总结

这是一个 **机器人伺服电机 PID 参数读取 / 设置工具**

作用：

**通过 CAN 总线读取指定电机的位置环、速度环、电流环 PID 参数，并可修改保存，用于电机调试、参数优化、机械臂运动稳定性调校。**

---

# 二、程序用途（非常明确）

1. **读取电机内部真实 PID 参数**
   * 位置环 P
   * 速度环 P / I
   * 电流环 P / I
2. **查看电机当前角度**
3. **可修改 PID 参数（代码里注释掉了）**
4. **可保存参数到电机 Flash**
5. **用于机械臂运动调试：变软、变硬、变稳、降噪、防抖动**

---

# 三、控制哪些电机？

代码里写死了：

cpp

运行

```
int id_array[] = {31,32,33,34, 41,42,43,44};
```

也就是：

* **31\~34：左臂肩部 4 个电机**
* **41\~44：右臂肩部 4 个电机**

---

# 四、逐模块超清晰解释

## 1. 头文件 + 全局电机 ID

cpp

运行

```
#include "ymbot_hardware_driver/ymbot_joint_eu.h"

int id_array[] = {31,32,33,34,41,42,43,44};
```

作用：

* 包含驱动库
* 指定要读取 PID 的**8 个肩部电机**

---

## 2. MotorFunction () 核心函数

### ① 初始化 CAN 总线

cpp

运行

```
planet_initDLL(...)
```

初始化 3 路 CAN 设备，用于连接电机。

### ② 给每个电机分配 CAN 通道

cpp

运行

```
if (id_array[i] >30 && <40) → dev_index=2
if (id_array[i] >40 && <50) → dev_index=0
```

根据电机 ID 自动分配对应 CAN 端口。

---

## 3. 核心功能：读取所有 PID 参数（最重要）

cpp

运行

```
planet_getPosition(...)                  // 当前角度
planet_getPOfPositionLoop(...)   // 位置环 P
planet_getPOfVelocityLoop(...)   // 速度环 P
planet_getIOfVelocityLoop(...)   // 速度环 I
planet_getPOfCurrentLoop(...)     // 电流环 P
planet_getIOfCurrentLoop(...)     // 电流环 I
```

程序会**依次打印每个电机的所有 PID 参数**。

---

## 4. 可修改 PID（代码里被注释了）

cpp

运行

```
// p_position = 64;
// planet_setPOfPositionLoop(...);
// planet_saveParas(...);
```

如果打开注释：

* 可设置 PID
* 可保存到电机内部

---

## 5. 释放 CAN 设备

cpp

运行

```
planet_freeDLL(0);
planet_freeDLL(1);
planet_freeDLL(2);
```

---

# 五、程序运行输出示例

plaintext

```
motor id: 31 position: 12.5
p of position loop: 100
p of velocity loop: 2000
i of velocity loop: 100
p of current loop: 120
i of current loop: 80
...
```

---

# 六、这个程序在系统里的作用（关键定位）

你之前的 4 个代码：

1. 键盘控制底盘
2. 底盘驱动
3. VR 控制关节
4. 键盘调试关节

**本段代码 = 电机底层参数调试工具**

它不控制运动，只**调试电机稳定性**：

* 机械臂抖动 → 降低 PID
* 机械臂无力 → 升高 PID
* 运动不顺畅 → 调 PID

---

# 七、总结

该程序是机器人**伺服电机 PID 参数调试工具**，通过 CAN 总线与伺服电机通信，实现位置环、速度环、电流环 PID 参数的读取与配置。

主要用于机械臂运动性能优化、振动抑制、响应速度调整，是提升机器人运动平稳性与控制精度的关键底层调试工具。
