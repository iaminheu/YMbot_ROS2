#### 编译说明

1. colcon build --symlink-install --packages-select chassis_driver_node


# 一、整体功能

这是一个 **ROS 2 机器人底盘驱动节点**，作用是：

**接收键盘 / 导航发来的速度指令 → 通过 TCP 网络发送给机器人底盘（云迹底盘）→ 让机器人真正动起来。**

# 二、它和上一段键盘代码的关系（超级重要）

上一段代码：**键盘控制节点** → 发布 `/key_cmd_vel` 速度指令

这段代码：**底盘驱动节点** → 订阅 `/key_cmd_vel` → 发给底盘硬件

它们是**上下游关系**：

**键盘节点 → 速度话题 → 底盘驱动节点 → TCP → 机器人底盘**

# 三、代码用途

1. 作为**机器人硬件驱动层**，对接真实机器人底盘
2. 接收 ROS 2 速度指令（键盘 / 导航 / AI 都可以）
3. 通过 **TCP 网络** 把指令转发给底盘控制器
4. 实现机器人真正的物理运动

# 四、核心技术

* 框架：**ROS 2**
* 语言：**C++**
* 通信：**TCP 客户端**（连接底盘控制器）
* 订阅话题：**/key\_cmd\_vel**（速度指令）
* 发送格式：**HTTP 风格 API 指令**
  `/api/joy_control?linear_velocity=xx&angular_velocity=xx`

# 五、逐模块详细解释

## 1. 宏定义

cpp

运行

```
#define CHASSIS_SEND_MSG_HZ 20
#define MAX_RECONNECT_ATTEMPTS 5
```

作用：

* 控制发送频率 **20Hz**
* 定义 TCP 重连次数、延迟

## 2. 类：ChassisDriverNode

底盘驱动主类，继承 ROS 2 节点。

## 3. 构造函数 + Init ()

cpp

运行

```
ChassisDriverNode()
{
    Init();
}

void Init()
{
    InitParams();         // 1. 加载参数
    CreateSubAndPub();    // 2. 创建订阅者
    InitTcpClient();      // 3. 连接底盘 TCP
    启动线程;
}
```

## 4. InitParams () —— 加载参数

从配置文件读取：

* TCP 服务器 IP：`192.168.10.10`
* 端口：`31001`
* 速度话题：`/key_cmd_vel`

## 5. CreateSubAndPub () —— 创建订阅者

cpp

运行

```
key_vel_control_sub_ = create_subscription<Twist>(
    "/key_cmd_vel", 1, KeyControlSubCallback);
```

作用：

**订阅键盘节点发来的速度指令**

## 6. InitTcpClient () —— 连接底盘 TCP

* 创建 socket
* 连接底盘控制器
* 成功打印：`链接tcp服务端成功`

## 7. ChassisTaskProcessThread () —— 发送线程

一个独立线程，**20 次 / 秒**不断从消息队列取出指令并发送 TCP。

## 8. SendTcpMessage () —— TCP 发送函数

真正把数据通过网络发给底盘。

## 9. InvokeKeyControl () —— 构造指令

把速度拼成云迹底盘支持的格式：

plaintext

```
/api/joy_control?angular_velocity=xx&linear_velocity=xx
```

## 10. KeyControlSubCallback () —— 回调函数（核心）

cpp

运行

```
void KeyControlSubCallback(const Twist::SharedPtr msg)
{
    float speed_val = msg->linear.x;
    float angle = msg->angular.z;
    InvokeKeyControl(speed_val, angle);
}
```

作用：

* 收到速度 → 直接转发给底盘
* 速度不变时不重复发送（优化）

## 11. main()

启动 ROS 2 节点，保持运行。

# 六、完整数据流（最清晰）

1. **键盘节点** 按 W/A/S/D → 发布 `/key_cmd_vel`
2. **底盘驱动节点** 订阅 `/key_cmd_vel`
3. 进入回调函数
4. 构造指令：`/api/joy_control?linear=xx&angular=xx`
5. 放入队列
6. 发送线程通过 **TCP** 发给底盘
7. **机器人底盘真正运动**

# 七、总结

该代码是 **ROS 2 机器人底盘驱动程序**，负责接收上层控制指令（键盘 / 导航），

通过 **TCP 网络通信** 将速度指令转发至云迹底盘控制器，实现机器人的运动控制。

它是**软件层与硬件层之间的关键桥梁**，完成从 ROS 指令到底盘硬件执行的完整链路。
