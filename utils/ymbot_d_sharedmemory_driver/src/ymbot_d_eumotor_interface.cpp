/*
 * 作者: chen furong
 * 创建日期: 2025.5.9
 * 描述: 使用共享内存驱动底盘型上肢
 * 版本: 1.1
 */

/* 注意事项
 * 确保urdf能正确使用，否则装配是有问题的，euDir的电机和urdf方向
 * sendCommands()函数在上机前要测试
 */

#include <cmath>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <mutex>
#include <iomanip>
#include <csignal>
#include "SharedMemoryArm.hpp"
#include "ymbot_joint_eu.h"
#include <filesystem>


struct MotorGroupConfig {
    int can_index;
    std::vector<int> motor_ids;
};

// 外部数据交换结构体
struct MotorData {
    std::vector<double> move_rad;              // 电机输入：移动弧度（rad）
    std::vector<double> current_position_rad;  // 电机输出：当前位置（rad）
    std::vector<double> current_velocity_rads; // 电机输出：当前速度（rad/s）
    std::vector<double> current_current_a;     // 电机输出：当前电流（a）
};

// -----------------------------------定义全局变量----------------------------------
EuArmData dataRes_[JOINT_ARM_NUMBER];     
double pos_des_arm_[JOINT_ARM_NUMBER];    
bool running = true;
volatile bool running_key = true;
void signalHandler(int) { running_key = false; }                      
size_t n_motor_group;                        
size_t n_motor;
            
std::vector<YmbotJointEu> motors;         

MotorData motor_data;
std::mutex data_mutex;
auto program_start = std::chrono::steady_clock::now();
bool enable_file = false; //默认不保存数据
std::ofstream motorsendtxt;
// 参考urdf中joint limit
// double euMaxPos[JOINT_ARM_NUMBER] = {         M_PI_2,           M_PI,   17*M_PI/180.0,  160*M_PI/180.0,         M_PI_2,  34*M_PI/180.0,
//                                        60*M_PI/180.0,  80*M_PI/180.0,  160*M_PI/180.0,   50*M_PI/180.0,  60*M_PI/180.0,   3*M_PI/180.0,  160*M_PI/180.0,
//                                       160*M_PI/180.0,  12*M_PI/180.0,  160*M_PI/180.0,   34*M_PI/180.0,  60*M_PI/180.0,  80*M_PI/180.0,  160*M_PI/180.0
//                                       };
// double euMinPos[JOINT_ARM_NUMBER] = {  -2*M_PI/180.0,  -2*M_PI/180.0,           -M_PI, -160*M_PI/180.0,        -M_PI_2, -55*M_PI/180.0,
//                                      -160*M_PI/180.0, -12*M_PI/180.0, -160*M_PI/180.0,  -38*M_PI/180.0, -60*M_PI/180.0, -81*M_PI/180.0, -160*M_PI/180.0,
//                                      -160*M_PI/180.0, -80*M_PI/180.0, -160*M_PI/180.0,  -50*M_PI/180.0, -60*M_PI/180.0,  -3*M_PI/180.0, -160*M_PI/180.0,
//                                      }; // 电机21老是低头，-34  
// double euMaxPos[JOINT_ARM_NUMBER] = {           1.57,           3.14,             0.3,        2.791111,           1.57,            0.6,
//                                          2.791111111,            1.4,     2.791111111,            0.87,    1.046666667,           0.03,  2.791111111,
//                                          2.791111111,           0.21,     2.791111111,             0.6,    1.046666667,            1.4,  2.791111111
//                                       };
// double euMinPos[JOINT_ARM_NUMBER] = {          -0.03,          -0.03,           -3.14,       -2.791111,          -1.57,           -0.6,
//                                         -2.791111111,          -0.21,    -2.791111111,            -0.6,   -1.046666667,           -1.4, -2.791111111,
//                                         -2.791111111,           -1.4,    -2.791111111,           -0.87,   -1.046666667,          -0.03, -2.791111111,
//                                      }; // 参考urdf
double euMaxPos[JOINT_ARM_NUMBER] = {           1.57,           3.14,             0.3,        2.791111,           1.57,            0.6,
                                         2.791111111,            1.4,     2.791111111,            0.87,    1.046666667,           0.03,  2.791111111,
                                         2.791111111,           0.21,     2.791111111,             0.8,    1.046666667,            1.4,  2.791111111
                                      };
double euMinPos[JOINT_ARM_NUMBER] = {          0.175,          -0.03,           -1.85,       -2.791111,          -1.57,           -1.84,
                                        -2.791111111,          -0.21,    -2.791111111,            -0.8,   -1.046666667,           -1.4, -2.791111111,
                                        -2.791111111,           -1.4,    -2.791111111,           -0.87,   -1.046666667,          -0.10, -2.791111111,
                                     }; // 参考urdf
double euDir[JOINT_ARM_NUMBER] = { 1, 1, 1, 1, 1,  1,
                                   1, 1, 1, 1, 1,  1, 1,
                                   1, 1, 1, 1, 1,  1, 1};      
double euBase[JOINT_ARM_NUMBER] = {65*M_PI/180.0, 155*M_PI/180.0, -M_PI_2,       0, 0, 0,
                                               0, -10*M_PI/180.0,       0,  M_PI_4, 0, 0, 0,
                                               0,  10*M_PI/180.0,       0, -M_PI_4, 0, 0, 0};

std::vector<MotorGroupConfig> load_motor_config(const std::string &config_path)
{
    std::vector<MotorGroupConfig> groups;
    YAML::Node config = YAML::LoadFile(config_path);
    
    for (const auto& group_node : config["motor_groups"]) {
        MotorGroupConfig group;
        group.can_index = group_node["can_index"].as<int>();
        group.motor_ids = group_node["ids"].as<std::vector<int>>();
        groups.push_back(group);
    }
    return groups;
}

// 修改 eumotor_interface.cpp 中的构造函数
void configMotors()
{
    // 从配置文件加载配置
    namespace fs = std::filesystem;
    // auto motor_groups = load_motor_config("/home/ymzz/YMbot_ROS2/utils/ymbot_d_sharedmemory_driver_new/config/motor_config.yaml");
    fs::path this_dir = fs::path(__FILE__).parent_path();  // parent_path: C++17 :contentReference[oaicite:1]{index=1}
    fs::path cfg = (this_dir / ".." / "config" / "motor_config.yaml").lexically_normal();

    auto motor_groups = load_motor_config(cfg.string());

    n_motor_group = motor_groups.size();
    // 创建所有电机实例
    for (const auto& group : motor_groups) {
        for (int motor_id : group.motor_ids) {
            YmbotJointEu motor;
            motor.motor_id = motor_id;
            motor.dev_index = group.can_index; // 直接使用配置的CAN口
            motors.push_back(motor);    // 电机顺序根据yaml
        }
    }
    n_motor = motors.size();
}

void disable() {
    // auto& instance = getInstance();
    // running = false;
    
    // 禁用所有电机
    for(auto& m : motors){
        m.motor_disabled();
    }
    
    // 关闭CAN通信
    for(size_t i=0; i<n_motor_group; ++i){
        planet_freeDLL(i);
    }
}

// bool saftCheck(EuArmData dataRes_[JOINT_ARM_NUMBER], MotorData motor_data)
// {
//     double error = 0.1;
//     for (size_t i = 0; i < JOINT_ARM_NUMBER; ++i)
//     {
//         if ((dataRes_[i].pos_ > (euMaxPos[i]+error)  )|| (dataRes_[i].pos_ < (euMinPos[i]-error) ))
//         {
//             std::cout << "[ymbot_d_eumotor_interface] " << " err id : " << motors[i].motor_id << "  pos  " << dataRes_[i].pos_ << "\n";
//             return false;
//         }
//     }
//     return true;
// }

bool saftCheck(double target_joint_rad[JOINT_ARM_NUMBER])
{
    double error = 0.0;
    for (size_t i = 0; i < JOINT_ARM_NUMBER; ++i)
    {
        if ((target_joint_rad[i] > (euMaxPos[i]+error) && motor_data.move_rad[i] > 0)|| (target_joint_rad[i] < (euMinPos[i]-error) && motor_data.move_rad[i] < 0))
        {
            std::cout << "[ymbot_d_eumotor_interface] " << " err id : " << motors[i].motor_id << "  pos  " << target_joint_rad[i] << " rad" << "\n";
            return false;
        }
    }
    return true;
}

void smoothHoming(YmbotJointEu& motor) {
    const double target_deg = 180.0;
    float current_deg = 0.0;  
    
    // 渐进式归位
    do {
        planet_getPosition(motor.dev_index, motor.motor_id, &current_deg);
        double step = (target_deg - current_deg) * 0.1; // 10%步进
        // 打印调试信息
        std::cout << "[ymbot_d_eumotor_interface] " << "Motor ID: " << motor.motor_id
                  << " | Current Position: " << current_deg << "°"
                  << " | Target Position: " << current_deg + step << "°"
                  << std::endl;
        // planet_quick_setTargetPosition(motor.dev_index, motor.motor_id, current_deg + step);
        // 快速直接归为180度（零位）
        // planet_quick_setTargetPosition(motor.dev_index, motor.motor_id, target_deg);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    } while(fabs(target_deg - current_deg) > 0.5);
}

void initMotors() {
    // 初始化CAN总线
    for(size_t dev=0; dev<n_motor_group; ++dev){

        // 先尝试关闭对应的CAN口  by cfr 2025.12.17
        std::string can_device = "can" + std::to_string(dev);
        std::string command = "sudo ip link set " + can_device + " down 2>/dev/null";
        
        std::cout << "[ymbot_d_eumotor_interface] Closing " << can_device << "..." << std::endl;
        int down_result = system(command.c_str());
        
        if (down_result != 0) {
            std::cout << "[ymbot_d_eumotor_interface] Warning: Failed to close " 
                      << can_device << " (maybe not exist or no permission)" << std::endl;
        }
        
        // 等待确保接口完全关闭
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if(planet_initDLL(planet_DeviceType_Canable, dev, 0, planet_Baudrate_1000) != PLANET_SUCCESS){ //CAN_SUCCESS
            std::cout << "[ymbot_d_eumotor_interface] " << "CAN" << dev << " initialization failed!" << std::endl;
            disable();
            std::exit(EXIT_FAILURE); // 立即退出程序
            // return;
        }
    }

    // 初始化电机
    for(auto& motor : motors){
        if(!motor.motor_initialization_CSP()){
            std::cout << "[ymbot_d_eumotor_interface] " << "Motor " << motor.motor_id << " initialization failed!" << std::endl;
            disable();
            std::exit(EXIT_FAILURE); // 立即退出程序
            // return;
        }

        // 步进运动到初始位姿
        // smoothHoming(motor);
    }

    // 初始化数据结构
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        motor_data.move_rad.resize(n_motor, 0.0);         // 初始增量为0
        motor_data.current_position_rad.resize(n_motor);
        motor_data.current_velocity_rads.resize(n_motor);
        motor_data.current_current_a.resize(n_motor);
    }
}

// 获取相对时间戳（毫秒）
uint64_t getRelativeTimestampMs() 
{
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        now - program_start).count();
}

void sendCommands()
{
    double motors_target[n_motor]; // 2025.5.20
    bool send_success = true; // 2025.5.20
    for(size_t i=0; i<n_motor; ++i){  // 控制yaml里所有电机
    // for(size_t i=0; i<n_motor-6; ++i){    // 只控制左右臂
        // 获取当前实际位置（单位：rad）
        double current_rad = motor_data.current_position_rad[i];
        // 计算绝对目标位置：current + delta
        double target_rad = current_rad + motor_data.move_rad[i];
        double target_deg = target_rad * 180.0 / M_PI;
        // std::cout << "[ymbot_d_eumotor_interface] " << "[Motor Control] Motor ID: " << motors[i].motor_id
        //           << " | Current Position: " << current_rad/M_PI*180.0 << "°"
        //           << " | Target Position: " << target_deg << "°"
        //           << std::endl;

        // -------------------------------------------测试前一定要注释！！！！-----------------------------
        if (planet_quick_setTargetPosition(motors[i].dev_index, motors[i].motor_id, target_deg) != PLANET_SUCCESS)
        {
            std::cout << "[ymbot_d_eumotor_interface] " << "Motor " << motors[i].motor_id << " send command failed!" << std::endl;
            send_success = false;
            break;
        }
        motors_target[i] = target_deg; //成功发送的数据
        motor_data.move_rad[i] = 0.0; // 清空增量
    }

    // 写入文件 2025.5.20
    if (enable_file && send_success){
        motorsendtxt << getRelativeTimestampMs() ;
    for (size_t i = 0; i < n_motor; ++i) {
        motorsendtxt << " " << motors_target[i];
      }
    motorsendtxt << std::endl;
    }
    
}

void receiveFeedback()
{
    for(size_t i=0; i<motors.size(); ++i){
        float deg, rpm, ma;
        if(planet_getPosition(motors[i].dev_index, motors[i].motor_id, &deg) == PLANET_SUCCESS // && //CAN_SUCCESS
           //planet_getVelocity(motors[i].dev_index, motors[i].motor_id, &rpm) == PLANET_SUCCESS &&
           //planet_getCurrent(motors[i].dev_index, motors[i].motor_id, &ma) == PLANET_SUCCESS
            ){
            // std::cout << "[ymbot_d_eumotor_interface] " <<  "Motor " << motors[i].motor_id << "deg" << deg << std::endl;
            // 单位转换
            motor_data.current_position_rad[i] = deg * M_PI / 180.0;
            //motor_data.current_velocity_rads[i] = rpm * 2 * M_PI / 60.0;
            //motor_data.current_current_a[i] = ma * 0.001;
        }
    }
}

// 生成标准化时间戳文件名 2025.5.20
std::string getTimestampFilename(const std::string& prefix, const std::string& suffix) {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now = *std::localtime(&time_t_now);
    
    std::ostringstream oss;
    oss << std::put_time(&tm_now, "%Y%m%d_%H%M%S");
    return prefix + "_" + oss.str() + suffix;
}

// 初始化日志文件 2025.5.20
void initdataFile() {
    if (!enable_file) return;
    std::string filename = getTimestampFilename("MotorSend", ".txt");
    motorsendtxt.open("../recorddata/"+filename, std::ios::out);
    if (!motorsendtxt.is_open()) {
        std::cout << "[ymbot_d_eumotor_interface] " << "[WARNING] Failed to open file: " << filename << std::endl;
    }
}

int analyzeParameters(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        // 处理 --record-data
        if (arg == "--record-data") {
            enable_file = true;
        } 
        
        // 处理其他参数
        // 帮助信息
        else if (arg == "--help") {
            std::cout << "[ymbot_d_eumotor_interface] " << "Usage: " << argv[0] << " [options]\n"
                      << "Options:\n"
                      << "  --record-data       Enable data logging (default)\n"
                      << "  --help              Show this help message\n";
            return 0;
        }
        
        // 无效参数
        else {
            std::cout << "[ymbot_d_eumotor_interface] " << "Unknown argument: " << arg << "\nUse --help for usage." << std::endl;
            return 0;
        }
    }
    return 1;
}

int main(int argc, char** argv)
{
    // 0.解析命令行参数
    if (analyzeParameters(argc, argv) != 1) return 0;

    // 1.加载yaml文件，配置电机ID和can_index
    configMotors();

    // 2.初始化can总线 电机使能
    initMotors();

    // 3.初始化共享内存（关节角度为0)，注意SharedMemoryArm.hpp文件里面的 JOINT_ARM_NUMBER 20 
    SharedMemoryArm shmArm(true); 
    // 新增标志：是否已写入初始状态
    bool initpos_written = false;
    
    // 4.控制循环的开关
    std::signal(SIGINT, signalHandler);  // 注册信号

    // 5.50Hz控制循环,并打开文件
    double maxDelta = 0.15; // 0.1---5 rad/s
    initdataFile(); // 2025.5.20

    auto loop_start = std::chrono::steady_clock::now();
    while (running_key)
    {
        // 延时
        loop_start = std::chrono::steady_clock::now();
        
        // 6.读取电机此刻的位置和速度,计算电机此刻的关节角度，并写入共享内存。
        receiveFeedback();

        // std::cout << "[ymbot_d_eumotor_interface] " << "用时1: " << elapsed.count() << std::endl;
        for (size_t i = 0; i < n_motor; ++i)
        {
            dataRes_[i].pos_ = (motor_data.current_position_rad[i] - M_PI) * euDir[i] + euBase[i];
            dataRes_[i].vel_ = motor_data.current_velocity_rads[i];
            dataRes_[i].cur_ = motor_data.current_current_a[i];
        }
        shmArm.writeJointDataArm(dataRes_);

        // std::cout << "[ymbot_d_eumotor_interface] " << "用时2: " << elapsed.count() << std::endl;

        // 将第一次拿到的位置，写入共享内存，避免共享内存的初始化位置0 (即使共享内存头文件不初始化，拿到的也是0)
        if (!initpos_written)
        {
            double initpos[JOINT_ARM_NUMBER];
            for(int i=0; i<JOINT_ARM_NUMBER; i++)
            {
                initpos[i] = dataRes_[i].pos_;
            }
            shmArm.writeJointDatatoMotorArm(initpos);
            initpos_written = true;
        }

        // 7.从共享内存取出目标关节角度，并跳过共享内存初始数据，计算移动弧度并限幅，下发给电机
        shmArm.readJointDataArm(pos_des_arm_);

        for (size_t i = 0; i < n_motor; ++i)
        {
            double delta = ((pos_des_arm_[i] - euBase[i]) * euDir[i] + M_PI) - motor_data.current_position_rad[i];
            if (delta > maxDelta)
            {
                delta = maxDelta;
            }
            else if (delta < -maxDelta)
            {
                delta = -maxDelta;
            }
            motor_data.move_rad[i] = delta;
            pos_des_arm_[i] = (delta + motor_data.current_position_rad[i] - M_PI) * euDir[i] + euBase[i];
            // std::cout << "[ymbot_d_eumotor_interface] " << "[sharedmemory read] Motor ID: " << motors[i].motor_id << " target joint pos: " << pos_des_arm_[i]*180.0/M_PI << std::endl;
        }
        // running = saftCheck(dataRes_, motor_data);   // 控制循环的开关：一旦此刻位置超出关节角度限位，
        running = saftCheck(pos_des_arm_);
        if (!running) continue;
        sendCommands();
        
        // 8.精确周期控制
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - loop_start);

        // std::cout << "[ymbot_d_eumotor_interface] " << "用时: " << elapsed.count() << std::endl;
        if (elapsed.count() < 20)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(20 - elapsed.count()));
        }
        else{
            std::cout << "[ymbot_d_eumotor_interface] " << "[ymbot_d_eumotor_interface] " << "[WARNING] Control loop over time: " << elapsed.count() << " ms" << std::endl;
        }
    }

    // 9.删除共享内存 2025.5.16
    shmArm.~SharedMemoryArm();
    shm_unlink(SHM_NAME_ARM);
    // 10.电机失能，关闭can通信
    disable();
    // 关闭文件 2025.5.20
    if (motorsendtxt.is_open()) motorsendtxt.close();
    return 0;
}