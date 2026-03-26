import socket
import json
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped, Pose, Twist
from std_msgs.msg import String, Float32MultiArray
import threading
import time

class ControllerPoseReceiver(Node):
    def __init__(self, port=7777):
        super().__init__('controller_pose_receiver')

        # 控制参数
        self.deadzone = 0.1 
        self.max_linear_speed = 0.2  # m/s
        self.max_angular_speed = 0.2  # rad/s
        self.torso_fixed_speed = 0.5  # 固定速度值

        # 复位参数 
        self._reset_cooldown = 1.0  # 1秒冷却时间，防止连续触发
        self._last_reset_time = 0.0  # 上次触发reset的时间

        # 底盘锁定
        self.chassis_locked = True
        self._prev_right_both = False  # 用于检测 right(btn1&btn2) 的按下沿
        self._prev_left_both = False   # 用于检测 left(btn1&btn2) 的按下沿
        
        # chord grace window
        self._chord_window_sec = 0.10   # 100ms，可调：0.06~0.12 常用
        self._up_grace_until = 0.0      # left.btn2/right.btn2 的等待窗截止
        self._down_grace_until = 0.0    # left.btn1/right.btn1 的等待窗截止
        self._prev_partial_up = False
        self._prev_partial_down = False

        # watchdog：防止断流还在走
        self._msg_timeout_sec = 0.5
        self.last_valid_msg_time = self.get_clock().now()

        # 当前命令缓存
        self._current_twist = Twist()
        self._current_torso_joints = [0.0] * 6  # 躯干关节命令

        # 手部控制参数
        self.grasp_safety_factor = 1  # 闭合安全系数（避免碰撞）
        self.current_grasp = [0.0, 0.0]  # [左手闭合度, 右手闭合度] (0.0=全开, 0.9=全闭)

        # 初始化发布者
        self.hand_grasp_pub = self.create_publisher(Float32MultiArray, "/hand_grasp", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.left_pose_publisher = self.create_publisher(Pose, "/arm_left/ee_status", 10)
        self.right_pose_publisher = self.create_publisher(Pose, "/arm_right/ee_status", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/key_cmd_vel", 10)
        self.torso_joints_pub = self.create_publisher(Float32MultiArray, "/torso_joints_vel", 1)
        self.reset_pub = self.create_publisher(String, '/reset_command', 10)

        # TCP通信初始化
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # 禁用Nagle
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 60)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 10)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
        self.sock.bind(("", port))
        self.sock.listen(1)
        self.connection = None

        self.get_logger().info(f"Waiting for TCP connection on port {port}...")

        # 定时器
        self._cmd_vel_timer = self.create_timer(0.1, self._publish_cmd_vel)  # 10Hz

        # 启动VR数据接收线程
        self._vr_thread = threading.Thread(target=self._vr_receive_thread, daemon=True)
        self._vr_thread.start()


    def _vr_receive_thread(self):  
        """VR数据接收线程"""
        while rclpy.ok():
            try:
                # 初始连接/重连
                if not self.connection:
                    if not self._reconnect():
                        self.get_logger().error("Reconnect failed, thread exiting")
                        return
                
                buffer = b""
                
                while rclpy.ok() and self.connection:
                    try:
                        # 阶段1：读取消息长度头（4字节）
                        while len(buffer) < 4:
                            try:
                                chunk = self.connection.recv(4 - len(buffer))
                                if not chunk:  # 连接关闭
                                    raise ConnectionResetError("Connection closed by peer")
                                buffer += chunk
                            except socket.timeout:
                                continue  # 正常超时，继续尝试
                        
                        msg_length = int.from_bytes(buffer[:4], byteorder='big')
                        if msg_length <= 0 or msg_length > 65536:
                            raise ValueError(f"Invalid message length: {msg_length}")
                        
                        # 阶段2：读取消息体
                        buffer = buffer[4:]  # 移除长度头
                        while len(buffer) < msg_length:
                            try:
                                remaining = msg_length - len(buffer)
                                chunk = self.connection.recv(min(4096, remaining))
                                if not chunk:
                                    raise ConnectionAbortedError("Incomplete message body")
                                buffer += chunk
                            except socket.timeout:
                                continue
                        
                        # 阶段3：处理完整消息
                        raw_message = buffer[:msg_length]
                        buffer = buffer[msg_length:]  # 移除已处理消息
                        
                        try:
                            message = raw_message.decode('utf-8').strip()
                            if message:  # 非空消息才处理
                                self._process_message(message)
                        except UnicodeDecodeError:
                            self.get_logger().warn(f"UTF-8 decode failed, discarded {len(raw_message)} bytes")
                        except Exception as e:
                            self.get_logger().error(f"Message processing error: {str(e)}")
                    
                    except (ConnectionResetError, ConnectionAbortedError) as e:
                        self.get_logger().warn(f"Connection error: {str(e)}")
                        if not self._reconnect():
                            break  # 跳出内层循环触发重连
                    
                    except ValueError as e:
                        self.get_logger().error(f"Protocol error: {str(e)}")
                        buffer = b""  # 清空缓冲区
                    
                    except Exception as e:
                        self.get_logger().error(f"Unexpected error: {str(e)}")
                        break
            
            except Exception as e:
                self.get_logger().error(f"Thread fatal error: {str(e)}", throttle_duration_sec=10)
            
            finally:
                # 清理连接
                if self.connection:
                    try:
                        self.connection.close()
                    except:
                        pass
                    finally:
                        self.connection = None
                
                # 等待后重试
                time.sleep(1)


    def _reconnect(self):
        """安全重连机制"""
        # 关闭现有连接
        if self.connection:
            try:
                self.connection.shutdown(socket.SHUT_RDWR)
                self.connection.close()
            except Exception as e:
                self.get_logger().warn(f"Connection close error: {str(e)}")
            finally:
                self.connection = None
        
        # 重试逻辑
        retry_count = 0
        max_retries = 5
        retry_delay = 1  # 初始延迟1秒
        
        while rclpy.ok() and retry_count < max_retries:
            try:
                self.get_logger().info(f"Reconnecting attempt {retry_count + 1}/{max_retries}...")
                self.connection, addr = self.sock.accept()
                self.connection.settimeout(1.0)
                self.get_logger().info(f"Successfully reconnected to {addr}")
                return True
            except socket.timeout:
                self.get_logger().warn("Connection timeout, retrying...")
            except Exception as e:
                self.get_logger().error(f"Reconnect failed: {str(e)}")
            
            retry_count += 1
            time.sleep(retry_delay)
            retry_delay = min(5, retry_delay * 2)  # 指数退避
        
        self.get_logger().error("Max reconnection attempts reached")
        return False
    

    def _process_message(self, message):
        message = message.strip()
        if not message:
            self.get_logger().debug("Received empty message")
            return None

        try:
            data = json.loads(message)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON decode failed: {str(e)}")
            return None

        left = None
        right = None

        # message 格式：{"left": "<dict>", "right": "<dict>"}
        if isinstance(data, dict) and ('left' in data or 'right' in data) and ('hand' not in data):
            for side in ('left', 'right'):
                payload = data[side]
                
                if isinstance(payload, dict):
                    payload.setdefault('hand', side)
                    if side == 'left':
                        left = payload
                    else:
                        right = payload
                else:
                    raise ValueError(f"Invalid payload type for {side} hand: {type(payload)}")
        else:
            raise ValueError("Invalid message format")

        if left is None and right is None:
            return None

        return self._process_dual_controllers(left, right)

    def _validate_controller_dict(self, data: dict):
        required_fields = {
            'hand': lambda x: x in ('left', 'right'),
            'pos': lambda x: isinstance(x, (list, tuple)) and len(x) == 3,
            'rot': lambda x: isinstance(x, (list, tuple)) and len(x) == 4,
            'btn1': lambda x: isinstance(x, bool),
            'btn2': lambda x: isinstance(x, bool),
            'index_trigger': lambda x: 0 <= float(x) <= 1,
            'hand_trigger': lambda x: 0 <= float(x) <= 1,
            'thumbstick': lambda x: isinstance(x, (list, tuple)) and len(x) == 2
        }
        for field, validator in required_fields.items():
            if field not in data:
                self.get_logger().error(f"[{data.get('hand','?')}] Missing required field: {field}")
                return False
            if not validator(data[field]):
                self.get_logger().error(f"[{data.get('hand','?')}] Invalid {field} value: {data[field]}")
                return False
        return True

    def _process_dual_controllers(self, left: dict | None, right: dict | None):
        """
        组合按键控制方式：
            index_trigger -> 手部开合
            hand_trigger -> TF + Pose

            左摇杆 -> 腰部
            右摇杆 -> 脖子

            左手 btn2 + 右手 btn2 躯干上升
            左手 btn1 + 右手 btn1 躯干下降

            右手 btn1+btn2 -> reset
            左手 btn1+btn2 -> 底盘锁定/解锁

            底盘按钮控制规则：
                1) 默认锁定（self.chassis_locked=True）
                2) 解锁后：
                    - only left.btn1 -> 前进
                    - only left.btn2 -> 后退
                    - only right.btn1 -> 左转
                    - only right.btn2 -> 右转
                3) 组合升降/右手双键/锁定状态 时，底盘速度强制 0
        """

        # 校验
        if left is not None and not self._validate_controller_dict(left):
            left = None
        if right is not None and not self._validate_controller_dict(right):
            right = None
        if left is None and right is None:
            return None

        now = self.get_clock().now()
        self.last_valid_msg_time = now

        # 提取数据，缺失则置默认
        # left
        l_btn1 = l_btn2 = False
        l_index = 0.0
        l_hand_trig = 0.0
        l_stick = (0.0, 0.0)
        l_pos = l_rot = None

        if left is not None:
            l_btn1 = bool(left['btn1'])
            l_btn2 = bool(left['btn2'])
            l_index = float(left['index_trigger'])
            l_hand_trig = float(left['hand_trigger'])
            l_stick = (float(left['thumbstick'][0]), float(left['thumbstick'][1]))
            l_pos = tuple(float(x) for x in left['pos'])
            l_rot = tuple(float(x) for x in left['rot'])

        # right
        r_btn1 = r_btn2 = False
        r_index = 0.0
        r_hand_trig = 0.0
        r_stick = (0.0, 0.0)
        r_pos = r_rot = None

        if right is not None:
            r_btn1 = bool(right['btn1'])
            r_btn2 = bool(right['btn2'])
            r_index = float(right['index_trigger'])
            r_hand_trig = float(right['hand_trigger'])
            r_stick = (float(right['thumbstick'][0]), float(right['thumbstick'][1]))
            r_pos = tuple(float(x) for x in right['pos'])
            r_rot = tuple(float(x) for x in right['rot'])

        # 右手 btn1+btn2：reset
        right_both = bool(r_btn1 and r_btn2)
        if right_both and (not self._prev_right_both):
            self._handle_reset_command()
        self._prev_right_both = right_both
        
        # 左手 btn1+btn2：底盘锁定/解锁
        left_both = bool(l_btn1 and l_btn2)
        if left_both and (not self._prev_left_both):
            self.chassis_locked = not self.chassis_locked
            self.get_logger().info(f"Chassis {'UNLOCKED' if not self.chassis_locked else 'LOCKED'} (toggle by left btn1+btn2)")
            
            # 切换为锁定时立即刹车
            if self.chassis_locked:
                self._current_twist.linear.x = 0.0
                self._current_twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self._current_twist)
        self._prev_left_both = left_both

        # 躯干控制
        joints = [0.0] * 6

        # 左摇杆 -> 腰部（2,3）
        lx = self._apply_deadzone(l_stick[0])
        ly = self._apply_deadzone(l_stick[1])
        joints[2] = ly * self.torso_fixed_speed
        joints[3] = lx * -self.torso_fixed_speed

        # 右摇杆 -> 脖子（4,5）
        rx = self._apply_deadzone(r_stick[0])
        ry = self._apply_deadzone(r_stick[1])
        joints[4] = rx * - (2*self.torso_fixed_speed)
        joints[5] = ry * - (2*self.torso_fixed_speed)

        # 组合键：left btn2 + right btn2 上升；left btn1 + right btn1 下降
        combo_up = bool(l_btn2 and r_btn2)
        combo_down = bool(l_btn1 and r_btn1)

        if combo_up:
            joints[0] = self.torso_fixed_speed
            joints[1] = self.torso_fixed_speed * 2
            joints[2] = -self.torso_fixed_speed
        elif combo_down:
            joints[0] = -self.torso_fixed_speed
            joints[1] = -self.torso_fixed_speed * 2
            joints[2] = self.torso_fixed_speed

        self._current_torso_joints = joints
        torso_msg = Float32MultiArray()
        torso_msg.data = self._current_torso_joints.copy()
        self.torso_joints_pub.publish(torso_msg)
        
        t = time.perf_counter()

        # 半组合键：只按下了一侧（并且至少有一侧为 True）
        partial_up = (l_btn2 != r_btn2) and (l_btn2 or r_btn2)      # 针对“上升”组合
        partial_down = (l_btn1 != r_btn1) and (l_btn1 or r_btn1)    # 针对“下降”组合

        # 只在底盘解锁时启用等待窗
        if not self.chassis_locked:
            # 仅在“半组合键的按下沿”开启等待窗，避免每帧刷新截止时间
            if partial_up and (not self._prev_partial_up) and (not combo_up):
                self._up_grace_until = t + self._chord_window_sec

                # 立刻刹车一次，防止上一帧速度残留继续跑
                self._current_twist.linear.x = 0.0
                self._current_twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self._current_twist)

            if partial_down and (not self._prev_partial_down) and (not combo_down):
                self._down_grace_until = t + self._chord_window_sec

                self._current_twist.linear.x = 0.0
                self._current_twist.angular.z = 0.0
                self.cmd_vel_pub.publish(self._current_twist)

        # 如果两侧都松开了，就清空等待窗，避免“空窗期”影响下一次操作
        if not (l_btn2 or r_btn2):
            self._up_grace_until = 0.0
        if not (l_btn1 or r_btn1):
            self._down_grace_until = 0.0

        self._prev_partial_up = partial_up
        self._prev_partial_down = partial_down

        chord_pending = (t < self._up_grace_until) or (t < self._down_grace_until)
                
        # 手部开合：左右 index_trigger
        if left is not None:
            self.current_grasp[0] = l_index * self.grasp_safety_factor
        if right is not None:
            self.current_grasp[1] = r_index * self.grasp_safety_factor

        grasp_msg = Float32MultiArray()
        grasp_msg.data = self.current_grasp.copy()
        self.hand_grasp_pub.publish(grasp_msg)

        # hand_trigger: TF + Pose
        if left is not None and l_pos is not None and l_rot is not None:
            tp = self.transform_position(l_pos)
            tr = self.transform_rotation(l_rot)
            self.publish_tf_transform('left', tp, tr)
            self.publish_pose('left', tp, tr, l_hand_trig)

        if right is not None and r_pos is not None and r_rot is not None:
            tp = self.transform_position(r_pos)
            tr = self.transform_rotation(r_rot)
            self.publish_tf_transform('right', tp, tr)
            self.publish_pose('right', tp, tr, r_hand_trig)

        #  底盘按钮离散控制
        # 规则：
        # 1) 默认锁定（self.chassis_locked=True）
        # 2) 解锁后：
        #    - only left.btn1 -> 前进
        #    - only left.btn2 -> 后退
        #    - only right.btn1 -> 左转（angular.z 正）
        #    - only right.btn2 -> 右转（angular.z 负）
        # 3) 组合升降/右手双键/锁定状态 时，底盘速度强制 0
        self._update_chassis_twist(
            l_btn1=l_btn1,
            l_btn2=l_btn2,
            r_btn1=r_btn1,
            r_btn2=r_btn2,
            block = combo_up or combo_down or right_both or left_both or chord_pending
        )

        return True

    # 控制输出
    def _update_chassis_twist(self, l_btn1: bool, l_btn2: bool, r_btn1: bool, r_btn2: bool, block: bool):
        if self.chassis_locked or block:
            self._current_twist.linear.x = 0.0
            self._current_twist.angular.z = 0.0
            return

        # 线速度
        if l_btn2 and (not l_btn1):
            self._current_twist.linear.x = self.max_linear_speed
        elif l_btn1 and (not l_btn2):
            self._current_twist.linear.x = -self.max_linear_speed
        else:
            self._current_twist.linear.x = 0.0

        # 角速度：左转(逆时针) 为正，右转(顺时针) 为负
        if r_btn2 and (not r_btn1):
            self._current_twist.angular.z = self.max_angular_speed
        elif r_btn1 and (not r_btn2):
            self._current_twist.angular.z = -self.max_angular_speed
        else:
            self._current_twist.angular.z = 0.0

    def _handle_reset_command(self):
        """处理reset命令，确保只发布一次"""
        current_time = time.perf_counter()

        # 检查是否已经触发过且仍在冷却期内
        if current_time - self._last_reset_time < self._reset_cooldown:
            return
        
        msg = String()
        msg.data = 'reset'
        self.reset_pub.publish(msg)
        self._last_reset_time = current_time
        self.get_logger().info("Reset command published")

    def _publish_cmd_vel(self):
        # watchdog：超过超时则刹车
        now = self.get_clock().now()
        dt_ns = (now - self.last_valid_msg_time).nanoseconds
        if dt_ns > int(self._msg_timeout_sec * 1e9):
            self._current_twist.linear.x = 0.0
            self._current_twist.angular.z = 0.0

        self.cmd_vel_pub.publish(self._current_twist)


    def publish_tf_transform(self, hand, position, rotation):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "base_link"
        transform.child_frame_id = f"controller_{hand}_frame"

        transform.transform.translation.x = position[0]
        transform.transform.translation.y = position[1]
        transform.transform.translation.z = position[2]

        transform.transform.rotation.x = -rotation[2]
        transform.transform.rotation.y = rotation[0]
        transform.transform.rotation.z = -rotation[1]
        transform.transform.rotation.w = rotation[3]

        self.tf_broadcaster.sendTransform(transform)

    def publish_pose(self, hand, position, rotation, hand_trigger: float):
        if not (hand_trigger > 0.5):
            return
        
        pose_msg = Pose()
        pose_msg.position.x = position[0]
        pose_msg.position.y = position[1]
        pose_msg.position.z = position[2]

        pose_msg.orientation.x = -rotation[2]
        pose_msg.orientation.y = rotation[0]
        pose_msg.orientation.z = -rotation[1]
        pose_msg.orientation.w = rotation[3]

        if hand == "left":
            self.left_pose_publisher.publish(pose_msg)
        elif hand == "right":
            self.right_pose_publisher.publish(pose_msg)

    def _apply_deadzone(self, value):
        """摇杆死区处理"""
        return 0.0 if abs(value) < self.deadzone else value

    @staticmethod
    def transform_position(position):
        x, y, z = position
        return (z, -x, y)

    @staticmethod
    def transform_rotation(rotation):
        return rotation

def main():
    rclpy.init()
    node = ControllerPoseReceiver(port=7777)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()