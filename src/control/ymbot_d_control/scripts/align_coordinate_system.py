# import numpy as np
# from scipy.spatial.transform import Rotation as R

# # 计算两个位姿之间的变换关系
# def compute_transformation(pose1, pose2):
#     # 位姿信息：[x, y, z, qx, qy, qz, qw]
#     pos1 = pose1[:3]  # 提取位置 (x, y, z)
#     quat1 = pose1[3:]  # 提取四元数 (qx, qy, qz, qw)
    
#     pos2 = pose2[:3]  # 提取位置 (x, y, z)
#     quat2 = pose2[3:]  # 提取四元数 (qx, qy, qz, qw)

#     # 计算平移变换 (Translation)
#     translation = np.subtract(pos2, pos1)
    
#     # 使用 scipy.spatial.transform 计算旋转变换 (Rotation)
#     # 使用 Rotation.from_quat 将四元数转换为 Rotation 对象
#     r1 = R.from_quat(quat1)  # quat1 是四元数 [qx, qy, qz, qw]
#     r2 = R.from_quat(quat2)  # quat2 是四元数 [qx, qy, qz, qw]
    
#     # 计算旋转变换 (q2 * q1^-1)
#     r_rel = r2 * r1.inv()  # 先求 quat1 的逆，然后计算 quat2 * quat1^-1
    
#     # 获取旋转后的四元数
#     quat_rel = r_rel.as_quat()  # 返回 [qx, qy, qz, qw] 顺序的四元数

#     return np.concatenate([translation, quat_rel])


# def align_coordinate_system(trans, pose_base):
#     pos_result = np.subtract(pose_base[0:3], trans[0:3])

#     r_trans = R.from_quat(trans[3:])
#     r_base = R.from_quat(pose_base[3:])
#     r_result = (r_base.inv() * r_trans).inv()
#     quat_result = r_result.as_quat()

#     return np.concatenate([pos_result, quat_result])



# # 主函数
# def main():
#     # 第一个位姿 [x, y, z, qx, qy, qz, qw]
#     pose1 = [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]  # 无旋转
    
#     # 第二个位姿 [x, y, z, qx, qy, qz, qw]
#     pose2 = [4.0, 5.0, 6.0, 0.0, 0.0, 0.707, 0.707]  # 旋转90度

#     pose3 = [6.0, 8.0, 19.0, 0.0, 0.0, 0.707, 0.707]  # 旋转90度
    
#     # 计算并输出变换关系
#     trans = compute_transformation(pose1, pose2)
#     print(f"Trans: ({trans[0]}, {trans[1]}, {trans[2]}, {trans[3]}, {trans[4]}, {trans[5]}, {trans[6]})")
 

#     result = align_coordinate_system(trans, pose2)
#     print(f"Result: ({result[0]}, {result[1]}, {result[2]}, {result[3]}, {result[4]}, {result[5]}, {result[6]})")

#     result = align_coordinate_system(trans, pose3)
#     print(f"Result: ({result[0]}, {result[1]}, {result[2]}, {result[3]}, {result[4]}, {result[5]}, {result[6]})")

# if __name__ == "__main__":
#     main()



import rosbags.rosbag2  # 使用正确的 rosbag2 库
import geometry_msgs.msg as geo_msgs

def extract_pose_data(bag_file, topics):
    # 使用 rosbags.rosbag2.Bag 读取 mcap 文件
    with rosbags.rosbag2.Bag(bag_file, 'r') as bag:
        data = {topic: [] for topic in topics}
        
        # 遍历所有消息
        for topic, msg, t in bag.messages(topics=topics):  # 使用 messages() 来读取消息
            if isinstance(msg, geo_msgs.PoseStamped):
                data[topic].append(msg.pose)  # 存储 pose 部分

        return data

def main():
    bag_file = "your_ros2_bag_file.mcap"  # 替换为您的 mcap 文件路径
    topics = [
        '/tf/Left_Arm_Link8/pose',
        '/tf/Right_Arm_Link8/pose',
        '/vrpn/left_arm/pose',
        '/vrpn/right_arm/pose'
    ]

    pose_data = extract_pose_data(bag_file, topics)

    # 打印提取的数据
    for topic, poses in pose_data.items():
        print(f"Topic: {topic}")
        for pose in poses:
            print(f"Position: {pose.position.x}, {pose.position.y}, {pose.position.z}")
            print(f"Orientation: {pose.orientation.x}, {pose.orientation.y}, {pose.orientation.z}, {pose.orientation.w}")
            print('-' * 40)

if __name__ == '__main__':
    main()
