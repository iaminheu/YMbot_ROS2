#include <iostream>
#include <thread>
#include <string>
#include <atomic>
#include <zmq.hpp>
#include <mutex>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <shared_mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <nlohmann/json.hpp>
#include <thread_safe_queue.hpp> // 线程安全队列
#include <yaml-cpp/yaml.h>

#include <thread_safe_queue.hpp> // 线程安全队列

namespace ymrobot
{
    class ChassisDriverNode : public rclcpp::Node
    {
    public:
        ChassisDriverNode();
        //*~ChassisDriverNode();

    private:
        void Init();

        void InitTcpClient();
        void InitParams();
        void CreateSubAndPub();

        void ChassisTaskProcessThread();    // 云迹底盘任务线程


        void KeyControlSubCallback(const geometry_msgs::msg::Twist::SharedPtr msg); // 键盘控制服务回调函数

        void SendTcpMessage(const std::string &msg);  // 发送云迹命令
        void AddTaskMessages(const std::string &msg); // 添加任务状态消息

        void InvokeKeyControl(const double &vel, const double &angle);

    private:
        std::thread receive_tcp_service_thread_, chassis_task_process_thread_, cloud_chassis_run_status_thread_;
        ThreadSafeQueue<std::string> cloud_chassis_task_queue_; // 云迹任务队列

        std::unique_ptr<zmq::context_t> tcp_context_;
        std::unique_ptr<zmq::socket_t> tcp_socket_;
        int tcp_socket_fd_;

        // sub / pub
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr key_vel_control_sub_{}; // 键盘控制订阅者

        std::string server_address_;
        int server_port_;
        std::string key_vel_move_base_topic_;
    };
}