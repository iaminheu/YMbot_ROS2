import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
import serial

from .utils.inspire_hand_new_api import (
    setspeed,
    setpower,
    setangle,
    get_actangle,
    get_actforce,
    clear_error,
    get_error,
)
import time

# 全局变量
right_hand_id = 0x01
left_hand_id = 0x01

# 手指完全张开和闭合的数值
FULLY_OPEN = 1000
FULLY_CLOSED = 0


class HandControlNode(Node):
    def __init__(self):
        super().__init__("hand_control_node")

        self.hand_grasp_sub = self.create_subscription(
            Float32MultiArray, "/hand_grasp", self.hand_grasp_callback_press, 10
        )

        # 灵巧手状态发布
        self.left_position_pub = self.create_publisher(
            Int32MultiArray, "/left_hand/joint_data", 10
        )
        self.left_force_pub = self.create_publisher(
            Int32MultiArray, "/left_hand/force", 10
        )
        self.right_position_pub = self.create_publisher(
            Int32MultiArray, "/right_hand/joint_data", 10
        )
        self.right_force_pub = self.create_publisher(
            Int32MultiArray, "/right_hand/force", 10
        )
        self.left_hand_target_pub = self.create_publisher(Int32MultiArray, "/left_hand/target", 1)
        self.right_hand_target_pub = self.create_publisher(Int32MultiArray, "/right_hand/target", 1)


        # 初始化左右手的串口通信
        self.ser_right = serial.Serial("/dev/ttyUSB1", 115200, timeout=1)
        self.ser_left = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)

        # 设置初始角度和速度
        self.set_initial_angles_and_speeds()

        # 初始动作target
        self.publish_hand_target("left", 0)
        self.publish_hand_target('right', 0)

        # 初始化力阈值
        self.right_power_threshold = (500, 500, 500, 500, 500, 250)  # 右手的力阈值
        self.left_power_threshold = (500, 500, 500, 500, 500, 250)  # 左手的力阈值

        # 设置力控
        setpower(self.ser_right, right_hand_id, *self.right_power_threshold)
        setpower(self.ser_left, left_hand_id, *self.left_power_threshold)

        # 定时器发布状态
        self.timer = self.create_timer(0.03, self.timer_callback)

        self.cur_time = time.perf_counter()

    def create_int32_array(self, data):
        msg = Int32MultiArray()
        msg.data = [int(x) for x in data]
        return msg

    def timer_callback(self):
        l_err = get_error(self.ser_left, left_hand_id)
        r_err = get_error(self.ser_right, right_hand_id)
        if any(l_err) == 1:
            clear_error(self.ser_left, left_hand_id)
            return
        if any(r_err) == 1:
            clear_error(self.ser_right, right_hand_id)
            return
        actual_angle_right = get_actangle(self.ser_right, right_hand_id)  # msg = [little, ring, middle, index, thumb_bend, thumb_rot]
        actual_force_right = get_actforce(self.ser_right, right_hand_id)

        actual_angle_left = get_actangle(self.ser_left, left_hand_id)
        actual_force_left = get_actforce(self.ser_left, left_hand_id)

        self.left_position_pub.publish(self.create_int32_array(actual_angle_left))
        self.left_force_pub.publish(self.create_int32_array(actual_force_left))
        
        self.right_position_pub.publish(self.create_int32_array(actual_angle_right))
        self.right_force_pub.publish(self.create_int32_array(actual_force_right))

    def set_initial_angles_and_speeds(self):
        # 设置初始角度（完全张开）
        setangle(
            self.ser_right,
            right_hand_id,
            FULLY_OPEN,
            FULLY_OPEN,
            FULLY_OPEN,
            FULLY_OPEN,
            FULLY_OPEN,
            50,
        )
        setangle(
            self.ser_left,
            left_hand_id,
            FULLY_OPEN,
            FULLY_OPEN,
            FULLY_OPEN,
            FULLY_OPEN,
            FULLY_OPEN,
            50,
        )
        # 设置初始速度
        setspeed(self.ser_right, right_hand_id, 500, 500, 500, 500, 500, 500)
        setspeed(self.ser_left, left_hand_id, 500, 500, 500, 500, 500, 500)

    def publish_hand_target(self, hand, target):
        tmsg = Int32MultiArray()
        tmsg.data = [target]
        if hand == "left":
            self.left_hand_target_pub.publish(tmsg)
        else:
            self.right_hand_target_pub.publish(tmsg)

    def hand_grasp_callback_click(self, msg):
        """处理双手扳机控制的回调函数"""
        if len(msg.data) != 2:
            self.get_logger().warn(
                f"Invalid hand_grasp message length: {len(msg.data)} (expected 2)"
            )
            return

        # 获取左右手扳机值
        left_grasp = msg.data[0]  # 左手扳机值 (0.0~1.0)
        right_grasp = msg.data[1]  # 右手扳机值 (0.0~1.0)

        TRIGGER_THRESHOLD = 0.5  # 扳机触发阈值
        if right_grasp <= TRIGGER_THRESHOLD and left_grasp <= TRIGGER_THRESHOLD:
            return
        if right_grasp > TRIGGER_THRESHOLD or left_grasp > TRIGGER_THRESHOLD:
            if time.perf_counter() - self.cur_time < 1.0:
                # self.get_logger().warn(f"Press too quick")
                return
            self.cur_time = time.perf_counter()
        self.get_logger().info(f"handle grasp {left_grasp = } {right_grasp = }")
        
        # ============== 参数配置 ==============
        # 力控阈值
        self.right_power_threshold = (500, 500, 300, 300, 300, 450)  # 右手力控
        self.left_power_threshold = (500, 500, 300, 300, 300, 450)  # 左手力控

        # 开合角度定义
        RIGHT_OPEN_ANGLES = (1000, 1000, 1000, 1000, 1000, 50)  # 右手张开
        RIGHT_CLOSE_ANGLES = (1000, 1000, 100, 500, 500, 50)  # 右手闭合
        LEFT_OPEN_ANGLES = (1000, 1000, 1000, 1000, 1000, 50)  # 左手张开
        LEFT_CLOSE_ANGLES = (1000, 1000, 100, 500, 500, 50)  # 左手闭合

        # 扳机阈值（可调整）

        # ============== 力控设置 ==============
        setpower(self.ser_right, right_hand_id, *self.right_power_threshold)
        setpower(self.ser_left, left_hand_id, *self.left_power_threshold)

        # ============== 右手控制逻辑 ==============
        if not hasattr(self, "right_hand_state"):
            self.right_hand_state = "releasing"  # 初始化状态

        # 扳机按下（>0.5）且当前状态为张开 → 闭合
        if right_grasp > TRIGGER_THRESHOLD and self.right_hand_state == "releasing":
            setangle(self.ser_right, right_hand_id, *RIGHT_CLOSE_ANGLES)
            self.right_hand_state = "press"
            self.publish_hand_target('right', 1)
            self.get_logger().info("Right hand: Closing (trigger pressed)")

        elif right_grasp > TRIGGER_THRESHOLD and self.right_hand_state == "press":
            setangle(self.ser_right, right_hand_id, *RIGHT_OPEN_ANGLES)
            self.right_hand_state = "releasing"
            self.publish_hand_target('right', 0)
            self.get_logger().info("Right hand: Opening (trigger released)")

        # ============== 左手控制逻辑 ==============
        if not hasattr(self, "left_hand_state"):
            self.left_hand_state = "releasing"  # 初始化状态

        # 扳机按下（>0.5）且当前状态为张开 → 闭合
        if left_grasp > TRIGGER_THRESHOLD and self.left_hand_state == "releasing":
            setangle(self.ser_left, left_hand_id, *LEFT_CLOSE_ANGLES)
            self.left_hand_state = "press"
            self.publish_hand_target('left', 1)
            self.get_logger().info("Left hand: Closing (trigger pressed)")

        elif left_grasp > TRIGGER_THRESHOLD and self.left_hand_state == "press":
            setangle(self.ser_left, left_hand_id, *LEFT_OPEN_ANGLES)
            self.left_hand_state = "releasing"
            self.publish_hand_target('left', 0)
            self.get_logger().info("Left hand: Opening (trigger released)")

    def hand_grasp_callback_press(self, msg):
        if len(msg.data) != 2:
            self.get_logger().warn(
                f"Invalid hand_grasp message length: {len(msg.data)} (expected 2)"
            )
            return

        left_grasp = float(msg.data[0])
        right_grasp = float(msg.data[1])

        PRESS_TH = 0.55     # 按下阈值
        RELEASE_TH = 0.45   # 松开阈值
        DEBOUNCE_S = 0.05   

        # 张开闭合角度
        RIGHT_OPEN_ANGLES = (1000, 1000, 1000, 1000, 1000, 50)
        RIGHT_CLOSE_ANGLES = (1000, 1000,  100,  500,  500, 50)
        LEFT_OPEN_ANGLES = (1000, 1000, 1000, 1000, 1000, 50)
        LEFT_CLOSE_ANGLES = (1000, 1000,  100,  500,  500, 50)

        # 初始化状态
        if not hasattr(self, "right_pressed"):
            self.right_pressed = False
            self.right_last_edge_t = 0.0
        if not hasattr(self, "left_pressed"):
            self.left_pressed = False
            self.left_last_edge_t = 0.0

        now = time.perf_counter()

        # 右手：长按保持闭合，松开张开 
        if not self.right_pressed:
            new_right_pressed = right_grasp > PRESS_TH
        else:
            new_right_pressed = right_grasp > RELEASE_TH

        if new_right_pressed != self.right_pressed and (now - self.right_last_edge_t) > DEBOUNCE_S:
            self.right_last_edge_t = now
            self.right_pressed = new_right_pressed

            if self.right_pressed:
                setangle(self.ser_right, right_hand_id, *RIGHT_CLOSE_ANGLES)
                self.publish_hand_target("right", 1)
                self.get_logger().info("Right hand: Closing (HOLD)")
            else:
                setangle(self.ser_right, right_hand_id, *RIGHT_OPEN_ANGLES)
                self.publish_hand_target("right", 0)
                self.get_logger().info("Right hand: Opening (RELEASE)")

        # 左手：长按保持闭合，松开张开
        if not self.left_pressed:
            new_left_pressed = left_grasp > PRESS_TH
        else:
            new_left_pressed = left_grasp > RELEASE_TH

        if new_left_pressed != self.left_pressed and (now - self.left_last_edge_t) > DEBOUNCE_S:
            self.left_last_edge_t = now
            self.left_pressed = new_left_pressed

            if self.left_pressed:
                setangle(self.ser_left, left_hand_id, *LEFT_CLOSE_ANGLES)
                self.publish_hand_target("left", 1)
                self.get_logger().info("Left hand: Closing (HOLD)")
            else:
                setangle(self.ser_left, left_hand_id, *LEFT_OPEN_ANGLES)
                self.publish_hand_target("left", 0)
                self.get_logger().info("Left hand: Opening (RELEASE)")

def main(args=None):
    rclpy.init(args=args)
    node = HandControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
