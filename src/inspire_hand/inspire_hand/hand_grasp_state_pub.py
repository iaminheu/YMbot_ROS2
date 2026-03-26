import numpy as np
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray, Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class HandGraspStatePub(Node):
    def __init__(self):
        super().__init__("hand_grasp_state_pub")

        # 订阅六维数据
        self.sub_l = self.create_subscription(
            Int32MultiArray, "/left_hand/joint_data", self.left_cb, 10
        )
        self.sub_r = self.create_subscription(
            Int32MultiArray, "/right_hand/joint_data", self.right_cb, 10
        )
        qos_latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # 发布state 0/1
        self.pub_l = self.create_publisher(Int32, "/left_hand/grasp01", qos_latched)
        self.pub_r = self.create_publisher(Int32, "/right_hand/grasp01", qos_latched)

        self.pub_left_angles = self.create_publisher(
            Float32MultiArray, "/left_hand/joint_angles", qos_latched
        )
        self.pub_right_angles = self.create_publisher(
            Float32MultiArray, "/right_hand/joint_angles", qos_latched
        )

        self.left_grasp01 = 0
        self.right_grasp01 = 0

        self.ABS_ON = 0.02
        self.SLOPE_ON = 0.35        # 上升沿阈值 (rad/s)
        self.SLOPE_OFF = 0.25       # 下降沿阈值 (rad/s)
        self.DELTA_ON = 0.015       # 上升沿阈值 (rad/frame)
        self.DELTA_OFF = 0.012      # 下降沿阈值 (rad/frame)
        self.DEBOUNCE_ON = 2
        self.DEBOUNCE_OFF = 2
        self.ALPHA_FILT = 0.25
        self._valid_idx = [0, 1, 2, 3, 5]  # 排除 thumb1

        self._l_state = None
        self._l_on_cnt = 0
        self._l_off_cnt = 0
        self._l_filt = None
        self._l_prev_filt = None
        self._l_prev_t = None

        self._r_state = None
        self._r_on_cnt = 0
        self._r_off_cnt = 0
        self._r_filt = None
        self._r_prev_filt = None
        self._r_prev_t = None

    @staticmethod
    def _to_rad_and_reorder(int_list_0_1000):
        angles_max = np.pi / 2
        tmp = angles_max - (np.array(int_list_0_1000, dtype=np.float32) * (angles_max / 1000.0))
        new_order = [3, 2, 0, 1, 5, 4]
        tmp = tmp[new_order]
        tmp[-2] = 1.52  # thumb1 固定
        return tmp  # (6,)

    def _update_grasp_state(self, raw6, side: str) -> int:
        x = raw6[self._valid_idx]  # (5,)

        if side == "L":
            state = self._l_state
            on_cnt, off_cnt = self._l_on_cnt, self._l_off_cnt
            filt, prev_filt = self._l_filt, self._l_prev_filt
            prev_t = self._l_prev_t
        else:
            state = self._r_state
            on_cnt, off_cnt = self._r_on_cnt, self._r_off_cnt
            filt, prev_filt = self._r_filt, self._r_prev_filt
            prev_t = self._r_prev_t
            
        now = time.perf_counter()
        
        # EMA 滤波
        if filt is None:
            filt = x.copy()
        else:
            filt = (1 - self.ALPHA_FILT) * filt + self.ALPHA_FILT * x

        level = float(np.max(filt))

        # 初始化
        if state is None:
            state = 1 if level >= self.ABS_ON else 0
            prev_filt = filt.copy()
            prev_t = now

        # # 用时间计算的方法 rad/s，目前不用
        # dt = now - prev_t
        # if dt <= 1e-4:
        #     dt = 1e-4
        # d = (filt - prev_filt) / dt  # (5,) rad/s

        # rise = float(np.max(d))
        # fall = float(np.min(d))
        # cond_on = (rise > self.SLOPE_ON) and (level > self.ABS_ON)
        # cond_off = (fall < -self.SLOPE_OFF)
        
        # rad/frame
        delta = filt - prev_filt
        rise = float(np.max(delta))
        fall = float(np.min(delta))

        # DELTA_ON = 0.015(rad/frame),  ABS_ON = 0.02,  DELTA_OFF = 0.012(rad/frame)
        cond_on = (rise > self.DELTA_ON) and (level > self.ABS_ON)
        cond_off = (fall < -self.DELTA_OFF)

        # if (rise > 0.007 and level > 0.02) or fall < -0.006:
        #     rise_symbol = True if rise > self.DELTA_ON else False
        #     level_symbol = True if level > self.ABS_ON else False
        #     fall_symbol = True if fall < -self.DELTA_OFF else False
        #     print(f'rise = {rise}-{rise_symbol}, level = {level}-{level_symbol}, fall = {fall}-{fall_symbol}, hand = {side}, time = {time.perf_counter()}') 
        # if cond_on:
        #     print("===================rise=====================")
        # if cond_off:
        #     print('+++++++++++++++++++fall+++++++++++++++++++++')

        if state == 0:
            on_cnt = on_cnt + 1 if cond_on else 0
            if on_cnt >= self.DEBOUNCE_ON:
                # print(f'============= 0 to 1 ===========')
                state = 1
                on_cnt = 0
                off_cnt = 0
        else:
            off_cnt = off_cnt + 1 if cond_off else 0
            if off_cnt >= self.DEBOUNCE_OFF:
                # print(f'+++++++++++++ 1 to 0+++++++++++')
                state = 0
                off_cnt = 0
                on_cnt = 0

        prev_filt = filt.copy()
        prev_t = now

        if side == "L":
            self._l_state = state
            self._l_on_cnt, self._l_off_cnt = on_cnt, off_cnt
            self._l_filt, self._l_prev_filt = filt, prev_filt
            self._l_prev_t = prev_t
        else:
            self._r_state = state
            self._r_on_cnt, self._r_off_cnt = on_cnt, off_cnt
            self._r_filt, self._r_prev_filt = filt, prev_filt
            self._r_prev_t = prev_t
            
        return int(state)

    def left_cb(self, msg: Int32MultiArray):
        raw6 = self._to_rad_and_reorder(msg.data)

        # 发布 joint_angles
        ang = Float32MultiArray()
        ang.data = raw6.astype(np.float32).tolist()   # 长度 6
        self.pub_left_angles.publish(ang)

        # 发布 grasp01
        self.left_grasp01 = self._update_grasp_state(raw6, side="L")
        out = Int32()
        out.data = self.left_grasp01
        self.pub_l.publish(out)

    def right_cb(self, msg: Int32MultiArray):
        raw6 = self._to_rad_and_reorder(msg.data)

        # 发布 joint_angles
        ang = Float32MultiArray()
        ang.data = raw6.astype(np.float32).tolist()   # 长度 6
        self.pub_right_angles.publish(ang)

        # 发布 grasp01
        self.right_grasp01 = self._update_grasp_state(raw6, side="R")
        out = Int32()
        out.data = self.right_grasp01
        self.pub_r.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = HandGraspStatePub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
