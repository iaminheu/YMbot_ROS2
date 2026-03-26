#include <rclcpp/rclcpp.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class JointJogController : public rclcpp::Node
{
public:
  JointJogController() : Node("body_joint_jog_tracking")
  {
    RCLCPP_INFO(this->get_logger(), "Body Joint Jog Node Initializing...");
  }
  void initialize() 
  {
    
    switch_input_client_ =this->create_client<moveit_msgs::srv::ServoCommandType>("/servo_node_body/switch_command_type");

    switch_servo_type2joint_jog();
    
    // 初始化Jog指令发布者
    jog_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
      "/servo_node_body/delta_joint_cmds", 10);

    // 订阅4维float数组指令（Body_Joint1-4速度）
    vel_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "vr_body_joints_vel_cmds", 10,
      std::bind(&JointJogController::velCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Body Joint Jog Controller Ready");
  }
private:

 bool switch_servo_type2joint_jog()
    {
        request_ = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
        request_->command_type = moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG;

        auto future = switch_input_client_->async_send_request(request_, 
            [this](rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedFuture response)
            {
                if (!response.get()->success)
                {
                    RCLCPP_WARN_STREAM(this->get_logger(), "Could not switch input to: JOINT_JOG");
                    return false;
                }
            });

        return true; // 直接返回，不阻塞当前线程
    }
  void velCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    // 验证数据长度
    
    if(msg->data.size() != 4) {
      RCLCPP_ERROR(this->get_logger(), 
          "Invalid command size! Expected 4, got %zu", msg->data.size());
      return;
    }

    // 创建JointJog消息
    auto jog_msg = control_msgs::msg::JointJog();


    
    // 设置关节名称（根据实际配置修改）
    jog_msg.joint_names = {
      "Body_Joint1", 
      "Body_Joint2",
      "Body_Joint3",
      "Body_Joint4"
    };

    // 转换float到double（Servo要求double类型）
    for(const auto& vel : msg->data) {
      jog_msg.velocities.push_back(static_cast<double>(vel));
    }

    // 设置时间戳和坐标系
    jog_msg.header.stamp = this->now();
    jog_msg.header.frame_id = "base_link"; // 根据实际坐标系修改

    // 发布指令
    jog_pub_->publish(jog_msg);

  }

  // 成员变量
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr jog_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr vel_sub_;
  std::shared_ptr<moveit_msgs::srv::ServoCommandType::Request> request_;
  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_input_client_;
};

int main(int argc, char** argv)
{

  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointJogController>();
  node->initialize();
  rclcpp::spin(node);
  //rclcpp::spin(std::make_shared<JointJogController>());
  // rclcpp::shutdown();
  // return 0;
}