# 灵巧手驱动包

包含了inspire灵巧手的python接口，可实现灵巧手电机初始化、读状态、写命令的操作  

# 1.使用教程

## 1.1 编译说明
1. conda activate ros2_py312
2. colcon build --symlink-install --packages-select vr_receiver_tcp

## 1.2 打开灵巧手的串口和指定手的id
示例：
```Python
ser_right = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
ser_left = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
right_hand_id = 0x01
left_hand_id = 0x01
```

## 1.3 初始化灵巧手各个电机的速度、力阈值与位置
启动程序后，可以先将灵巧手归位到自己期望的位置，并设置力、速度阈值。这三项的范围均是 0 ~ 1000
示例：
```Python
# 设置初始角度
setangle(ser_right, right_hand_id, 1000, 1000, 1000, 1000, 1000, 50)  # 设置右手初始角度 
setangle(ser_left, left_hand_id, 1000, 1000, 1000, 1000, 1000, 50)    # 设置左手初始角度
# 设置力阈值
setpower(ser_right, right_hand_id, 300, 300, 300, 300, 300, 300)  # 设置右手力阈值
setpower(ser_left, left_hand_id, 300, 300, 300, 300, 300, 300)  # 设置左手力阈值
# 设置初始速度
setspeed(ser_right, right_hand_id, 100, 100, 100, 100, 100, 100)  # 设置右手初始速度
setspeed(ser_left, left_hand_id, 100, 100, 100, 100, 100, 100)    # 设置左手初始速度
```
## 1.4 执行位置指令
需要灵巧手执行动作时，使用setangle进行控制命令的下发，在灵巧手到达位置过程中，如果某个电机的力超过了所设力阈值，该电机将会自动停在此处
示例：
```Python
# 设置手合拢
setangle(ser_right, right_hand_id, 100, 100, 100, 400, 700, 50)  # 右手合拢
setangle(ser_left, left_hand_id, 100, 100, 100, 400, 700, 50)    # 左手合拢

# 休眠1秒
time.sleep(1)

setangle(ser_right, right_hand_id, 1000, 1000, 1000, 1000, 1000, 50)  # 右手张开
setangle(ser_left, left_hand_id, 1000, 1000, 1000, 1000, 1000, 50)    # 左手张开
```

# 2.待优化 TODO:

1. 现在开发手的固定动作非常花费时间
通过不同任务的yaml来定义固定任务

