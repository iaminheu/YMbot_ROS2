/*
 * 作者: chen furong
 * 创建日期: 2025.5.12
 * 描述: 测试使用共享内存驱动底盘型上肢
 * 版本: 1.0
 */

#include <iostream>
#include <cmath>
#include <vector>
#include <chrono>
#include <thread>
#include "SharedMemoryArm.hpp"

EuArmData armRes_[JOINT_ARM_NUMBER];
// std::vector<YmbotJointEu> joints;     // 定义控制端joint类

// 设置测试数据（使用安全范围内的值）
std::vector<std::vector<double>> joints = {
    // 左臂
     {-33, 39, 85, 8, -20, -19, -20, 27, -47, -11, 8, -39, 44, -23}  // 第一组关节数据
    ,{  0,  0,  0, 0,   0,   0,   0,  0,   0,   0, 0,   0,  0,   0}              // 第二组关节数据
};

int main() {
    
    // 初始化共享内存
    SharedMemoryArm shmArm(false);
    double expected_pos[JOINT_ARM_NUMBER];

    size_t j = 0;
    while (j < joints.size())
    {
        // 从共享内存中读取电机此刻数据，并跳过共享内存数据初始数据
        shmArm.readJointDatafromMotorArm(armRes_);
        if (armRes_[0].pos_ == 0){
            std::cout << "[Shared memory] has not updated motor data, check eumotor driver..." << std::endl;
            continue;
        }
        for (int i = 0; i < JOINT_ARM_NUMBER; ++i)
        { 
            std::cout << "[sharedmemory read] Motor index: " << i << 
            " current joint pos: " << armRes_[i].pos_*180.0/M_PI <<  " du vel: " << armRes_[i].vel_ << " rad/s  current " << armRes_[i].cur_ << "A" << std::endl;
            expected_pos[i] = armRes_[i].pos_;
        }

        // 组织目标关节角度，写入共享内存。
        for (size_t i = 0; i < joints[0].size(); ++i)
        {
            expected_pos[6+i] = joints[j][i]*M_PI/180.0;
        }
        for (size_t i = 0; i < JOINT_ARM_NUMBER; ++i)
        { 
            std::cout << "[sharedmemory input] Motor index: " << i << 
            " expected joint pos: " << expected_pos[i]*180.0/M_PI << "" << std::endl;
        }
        shmArm.writeJointDatatoMotorArm(expected_pos);

        j=j+1;
        std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    }
    
    return 0;
}