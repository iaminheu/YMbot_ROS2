#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

const char *green = "\033[1;32m";
const char *reset = "\033[0m";

class NeckJointPublisher : public rclcpp::Node {
public:
  NeckJointPublisher()
      : Node("position_tracking_neck"), target_angle_joint1_(M_PI / 0.0),
        target_angle_joint2_(M_PI / 0.0), radians_joint1_(0.0),
        radians_joint2_(0.0), prev_radians_joint1_(0.0),
        prev_radians_joint2_(0.0), got_joint_states_(false), duration_(3.0),
        initial_joint1_(0.0), initial_joint2_(0.0),
        got_initial_joint_states_(false), ee_joint1_(0.0), ee_joint2_(0.0),
        got_ee_status_(false) {
    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/neck_controller/commands", 10);
    joint_state_subscriber_ =
        this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&NeckJointPublisher::joint_state_callback, this,
                      std::placeholders::_1));
    ee_status_subscriber_ =
        this->create_subscription<sensor_msgs::msg::JointState>(
            "/neck/ee_status", 10,
            std::bind(&NeckJointPublisher::ee_status_callback, this,
                      std::placeholders::_1));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&NeckJointPublisher::publish_joint_trajectory, this));

    // 关节限位
    neck_joint1_min_ = -1.57;
    neck_joint1_max_ = 1.57;
    neck_joint2_min_ = -0.6;
    neck_joint2_max_ = 0.6;
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->position.size() >= 7 && !got_initial_joint_states_) {
      initial_joint1_ = msg->position[2]; // 第三个元素
      initial_joint2_ = msg->position[6]; // 第七个元素
      got_initial_joint_states_ = true;
      RCLCPP_INFO(
          this->get_logger(),
          "%sReceived initial neck joint states: Joint1: %f, Joint2: %f%s",
          green, initial_joint1_, initial_joint2_, reset);
    }
  }

  void ee_status_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->position.size() >= 2) {
      ee_joint1_ = msg->position[0];
      ee_joint2_ = msg->position[1];
      got_ee_status_ = true;
    }
  }

  void publish_joint_trajectory() {
    if (!got_initial_joint_states_ || !got_ee_status_) {
      return; // 等待接收到初始关节状态和末端执行器状态
    }

    double new_joint1 = initial_joint1_ + ee_joint1_;
    double new_joint2 = initial_joint2_ + ee_joint2_;

    // 检查关节限位
    if (new_joint1 < neck_joint1_min_ || new_joint1 > neck_joint1_max_) {
      new_joint1 = prev_radians_joint1_;
    } else {
      prev_radians_joint1_ = new_joint1;
    }

    if (new_joint2 < neck_joint2_min_ || new_joint2 > neck_joint2_max_) {
      new_joint2 = prev_radians_joint2_;
    } else {
      prev_radians_joint2_ = new_joint2;
    }

    auto message = std_msgs::msg::Float64MultiArray();
    message.data = {new_joint1, new_joint2};

    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      ee_status_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  double target_angle_joint1_;
  double target_angle_joint2_;
  double radians_joint1_;
  double radians_joint2_;
  double prev_radians_joint1_;
  double prev_radians_joint2_;
  bool got_joint_states_;
  double duration_;
  double initial_joint1_;
  double initial_joint2_;
  bool got_initial_joint_states_;
  double ee_joint1_;
  double ee_joint2_;
  bool got_ee_status_;
  double neck_joint1_min_;
  double neck_joint1_max_;
  double neck_joint2_min_;
  double neck_joint2_max_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NeckJointPublisher>());
  rclcpp::shutdown();
  return 0;
}