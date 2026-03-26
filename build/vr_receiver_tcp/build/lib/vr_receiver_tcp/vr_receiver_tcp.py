import socket
import json
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped, Pose, Twist
from std_msgs.msg import String, Float32MultiArray, Int32MultiArray, Int32
import threading
import time

class ControllerPoseReceiver(Node):
    def __init__(self, port=7777):
        super().__init__('controller_pose_receiver')

        # 控制参数
        self.deadzone = 0.1 
        self.max_linear_speed = 0.1  # m/s
        self.max_angular_speed = 0.2  # rad/s
        self.torso_fixed_speed = 4.0  # 固定速度值

        # 初始化控制命令
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

        # 记录上次的按键扳机状态
        self.last_button_state = {"left": {}, "right": {}}
        
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

        self.last_valid_msg_time = self.get_clock().now()
        self.current_mode = "Chassis Control"
        self._last_left_buttons = (False, False)

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


    def _process_message(self, message):
        """完整消息处理方法"""
        # 第一步：消息预处理
        message = message.strip()
        if not message:
            self.get_logger().debug("Received empty message")
            return None
        
        # 第二步：JSON解析（带修复尝试）
        try:
            # 尝试直接解析
            data = json.loads(message)
        except json.JSONDecodeError as e:
            # 修复尝试1：检查是否有多个JSON对象粘连
            if '}{' in message:
                parts = message.split('}{')
                message = parts[0] + '}'
                try:
                    data = json.loads(message)
                    self.get_logger().warn("Fixed concatenated JSON messages")
                except json.JSONDecodeError:
                    pass
            
            # 修复尝试2：截断到错误位置
            if 'data' not in locals():
                try:
                    message = message[:e.pos] + '}' if e.pos > len(message)//2 else message
                    data = json.loads(message)
                    self.get_logger().warn("Fixed truncated JSON message")
                except json.JSONDecodeError:
                    self.get_logger().error(f"JSON decode failed: {str(e)}")
                    return None
        
        # 第三步：字段验证
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
                self.get_logger().error(f"Missing required field: {field}")
                return None
            if not validator(data[field]):
                self.get_logger().error(f"Invalid {field} value: {data[field]}")
                return None
        
        # 第四步：数据提取
        try:
            hand = data['hand']
            position = tuple(float(x) for x in data['pos'])
            rotation = tuple(float(x) for x in data['rot'])
            btn1 = bool(data['btn1'])
            btn2 = bool(data['btn2'])
            index_trigger = float(data['index_trigger'])
            hand_trigger = float(data['hand_trigger'])
            thumbstick = tuple(float(x) for x in data['thumbstick'])
            mode = data.get('mode', 'Chassis Control')
            
            # 更新控制模式
            self.current_mode = mode
            
            # 坐标变换
            transformed_position = self.transform_position(position)
            transformed_rotation = self.transform_rotation(rotation)
            
            # 发布TF和位姿
            self.publish_tf_transform(hand, transformed_position, transformed_rotation)
            self.publish_pose(hand, transformed_position, transformed_rotation)
            
            # 控制逻辑路由
            self.control_mode_routing(hand, btn1, btn2, index_trigger, hand_trigger, thumbstick)
            
        except Exception as e:
            self.get_logger().error(f"Data processing error: {str(e)}")
            return None
        
        return True
    
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
    
    def control_mode_routing(self, hand, btn1, btn2, index_trigger, hand_trigger, thumbstick):
        """控制模式路由"""
        self.last_button_state[hand] = {
            "btn1": btn1,
            "btn2": btn2,
            "index_trigger": index_trigger,
            "hand_trigger": hand_trigger,
            "thumbstick_x": thumbstick[0],
            "thumbstick_y": thumbstick[1]
        }

        if not hasattr(self, 'current_mode'):
            return
            
        if self.current_mode == "Chassis Control":
            self._process_chassis_control(hand, thumbstick)
            
        elif self.current_mode == "Torso Control":
            self._process_torso_control(hand, btn1, btn2, thumbstick)


        self._hand_grasp(hand, index_trigger)


    def _process_chassis_control(self, hand, thumbstick):
        """更新底盘控制数据"""
        if hand == "left":
            y_axis = self._apply_deadzone(thumbstick[1])
            self._current_twist.linear.x = y_axis * self.max_linear_speed
            
        elif hand == "right":
            x_axis = self._apply_deadzone(thumbstick[0])
            self._current_twist.angular.z = x_axis * -self.max_angular_speed

        # self.cmd_vel_pub.publish(self._current_twist)

    def _process_torso_control(self, hand, btn1, btn2, thumbstick):
        """更新躯干控制数据"""
        
        if hand == "left":
            # 左摇杆控制腰部
            self._current_torso_joints[2] = self._apply_deadzone(thumbstick[1]) * self.torso_fixed_speed  # 前后
            self._current_torso_joints[3] = self._apply_deadzone(thumbstick[0]) * -self.torso_fixed_speed  # 左右
            self._last_left_buttons = (btn1, btn2)  # 记录按键状态
            
        elif hand == "right":
            # 右摇杆控制脖子
            self._current_torso_joints[4] = self._apply_deadzone(thumbstick[0]) * -self.torso_fixed_speed  # 左右
            self._current_torso_joints[5] = self._apply_deadzone(thumbstick[1]) * self.torso_fixed_speed  # 前后
            
            # 组合按键检测（需左右手数据）
            if hasattr(self, '_last_left_buttons'):
                left_x, left_y = self._last_left_buttons
                if left_y and btn2:  # Y+B 上升
                    self._current_torso_joints[0] = self.torso_fixed_speed 
                    self._current_torso_joints[1] = self.torso_fixed_speed * 2
                    self._current_torso_joints[2] = -self.torso_fixed_speed
                elif left_x and btn1:  # X+A 下降
                    self._current_torso_joints[0] = -self.torso_fixed_speed 
                    self._current_torso_joints[1] = -self.torso_fixed_speed * 2
                    self._current_torso_joints[2] = self.torso_fixed_speed

                else:
                    self._current_torso_joints[0] = 0.0
                    self._current_torso_joints[1] = 0.0
                    self._current_torso_joints[2] = 0.0
        
        msg = Float32MultiArray(data=self._current_torso_joints.copy())
        self.torso_joints_pub.publish(msg)


    def _hand_grasp(self, hand, index_trigger):
        """实时同步手部开合状态"""
        hand_idx = 0 if hand == "left" else 1
        self.current_grasp[hand_idx] = index_trigger * self.grasp_safety_factor
        
        msg = Float32MultiArray()
        msg.data = self.current_grasp.copy()
        self.hand_grasp_pub.publish(msg)


    def _publish_cmd_vel(self):
        """定时发布底盘控制命令"""
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

    def publish_pose(self, hand, position, rotation):
        if not self.last_button_state[hand].get("hand_trigger", 0) > 0.5:
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
    def parse_pose_data(data):
        """解析JSON数据并返回元组 (hand, position, rotation, btn1, btn2, index_trigger, hand_trigger, thumbstick, mode)"""
        try:
            parsed = json.loads(data)
            return (
                parsed["hand"],
                tuple(float(x) for x in parsed["pos"]),
                tuple(float(x) for x in parsed["rot"]),
                parsed["btn1"],
                parsed["btn2"],
                float(parsed["index_trigger"]),
                float(parsed["hand_trigger"]),
                tuple(float(x) for x in parsed["thumbstick"]),
                parsed.get("mode", "Chassis Control"), 
            )
        except Exception as e:
            print(f"Parse error: {e}")
            return None

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