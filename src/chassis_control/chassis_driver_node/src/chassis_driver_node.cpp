#include <chassis_driver_node.hpp>
#define CHASSIS_SEND_MSG_HZ 20   // 云迹底盘发送消息频率
#define CHASSIS_NAV_RESULST_HZ 1 // 云迹底盘发布导航结果频率
#define CHASSIS_STATUS_HZ 1      // 云迹底盘状态更新线程

#define MAX_RECONNECT_ATTEMPTS 5        // 最大重连次数
#define INITIAL_RECONNECT_DELAY_MS 1000 // 初始重连延迟（毫秒）
#define MAX_RECONNECT_DELAY_MS 10000    // 最大重连延迟（毫秒）

namespace ymrobot
{
    ChassisDriverNode::ChassisDriverNode() : Node("chassis_driver_node")
    {
        Init();
    }

    void ChassisDriverNode::Init()
    {
        InitParams();
        CreateSubAndPub();
        InitTcpClient();
        chassis_task_process_thread_ = std::thread(&ChassisDriverNode::ChassisTaskProcessThread, this);
    }

    void ChassisDriverNode::InitParams()
    {
        this->declare_parameter("server_address", "192.168.10.10");
        this->declare_parameter("server_port", 31001);
        this->declare_parameter("key_vel_move_base_topic", "/key_cmd_vel");

        server_address_ = this->get_parameter("server_address").as_string();
        server_port_ = this->get_parameter("server_port").as_int();
        key_vel_move_base_topic_ = this->get_parameter("key_vel_move_base_topic").as_string();
    }

    void ChassisDriverNode::CreateSubAndPub()
    {
        key_vel_control_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/key_cmd_vel", 1, std::bind(&ChassisDriverNode::KeyControlSubCallback, this, std::placeholders::_1));
        // std::cout << "22222"<<std::endl;
    }

    void ChassisDriverNode::InitTcpClient()
    {
        tcp_socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (tcp_socket_fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "无法连接到tcp服务端");
            return;
        }

        struct sockaddr_in server_addr;
        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(server_port_); // tcp端口号
        RCLCPP_INFO(this->get_logger(), "tcp服务端地址：%s", server_address_.c_str());
        RCLCPP_INFO(this->get_logger(), "tcp服务端端口：%d", server_port_);

        if (inet_pton(AF_INET, server_address_.c_str(), &server_addr.sin_addr) <= 0)
        {
            RCLCPP_ERROR(this->get_logger(), "无效的IP地址");
            close(tcp_socket_fd_);
            return;
        }

        if (connect(tcp_socket_fd_, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "连接到tcp服务端失败");
            close(tcp_socket_fd_);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "链接tcp服务端成功,服务端地址和端口号分别为: %s:%d", server_address_.c_str(), server_port_);
    }

    void ChassisDriverNode::ChassisTaskProcessThread()
    {
        rclcpp::WallRate loop_rate(CHASSIS_SEND_MSG_HZ);
        while (rclcpp::ok())
        {
            // 优先处理事件驱动消息
            std::string message;
            while (cloud_chassis_task_queue_.TryPop(message))
            {
                SendTcpMessage(message);
                message.clear();
            }
            loop_rate.sleep();
        }
    }

    void ChassisDriverNode::SendTcpMessage(const std::string &msg)
    {
        if (msg.empty())
        {
            RCLCPP_INFO(this->get_logger(), "SendTcpMessage msg is empty");
            return;
        }

        try
        {
            // std::lock_guard<std::mutex> l2(tcp_socket_mutex_);
            if (tcp_socket_fd_ < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Socket is not connected");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "发送消息: %s", msg.c_str());

            if (send(tcp_socket_fd_, msg.c_str(), msg.size(), 0) < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to send message");
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Unexpected error: %s", e.what());
        }
    }

    void ChassisDriverNode::AddTaskMessages(const std::string &msg)
    {
        cloud_chassis_task_queue_.Push(msg);
    }

    void ChassisDriverNode::InvokeKeyControl(const double &vel, const double &angle)
    {
        std::string cmd = "/api/joy_control?";
        std::string vel_msg = "&linear_velocity=" + std::to_string(vel);
        std::string angle_msg = "angular_velocity=" + std::to_string(angle);
        std::string send_msg = cmd + angle_msg + vel_msg;
        AddTaskMessages(send_msg);
    }

    void ChassisDriverNode::KeyControlSubCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        float speed_val = msg->linear.x;
        float angle = msg->angular.z;
        static float last_speed = 0.0;
        static float last_angle = 0.0;
        if(fabs(speed_val - last_speed) < 1e-6 && fabs(angle - last_angle) < 1e-6)
        {
            return;
        }
        // std::cout << "CSHU : "<< speed_val << "    "<< angle<<std::endl;
        InvokeKeyControl(speed_val, angle);
    }
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ymrobot::ChassisDriverNode>());
    rclcpp::shutdown();
    return 0;
}