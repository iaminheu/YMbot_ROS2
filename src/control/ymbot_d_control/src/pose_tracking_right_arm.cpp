/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*      Title       : pose_tracking_tutorial.cpp
 *      Project     : moveit2_tutorials
 *      Created     : 08/07/2023
 *      Author      : V Mohammed Ibrahim
 *
 *      Description : Example of using pose tracking via the ROS API in a door
 * opening scenario.
 */

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <moveit_servo/utils/common.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <moveit_msgs/srv/servo_command_type.hpp>
#include <moveit_msgs/msg/servo_status.hpp>

bool first_pose_received = false;
geometry_msgs::msg::PoseStamped first_current_pose;
geometry_msgs::msg::PoseStamped target_pose;

// 作为servo急停的标志位
bool is_servo_halt = false;

geometry_msgs::msg::Pose first_pose;
void vrPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
  if (!first_pose_received) {
    first_pose = *msg;
    first_pose_received = true;
    RCLCPP_INFO(rclcpp::get_logger("pose_tracking_right_arm"),
                "First pose received.");
  } else {
    geometry_msgs::msg::Pose relative_pose;
    relative_pose.position.x = msg->position.x - first_pose.position.x;
    relative_pose.position.y = msg->position.y - first_pose.position.y;
    relative_pose.position.z = msg->position.z - first_pose.position.z;

    tf2::Quaternion q1(msg->orientation.x, msg->orientation.y,
                       msg->orientation.z, msg->orientation.w);
    tf2::Quaternion q2(first_pose.orientation.x, first_pose.orientation.y,
                       first_pose.orientation.z, first_pose.orientation.w);
    tf2::Quaternion relative_orientation = q1 * q2.inverse();

    // 将 relative_pose 加至 current_pose 上，作为新的目标位姿
    target_pose.pose.position.x =
        first_current_pose.pose.position.x + relative_pose.position.x;
    target_pose.pose.position.y =
        first_current_pose.pose.position.y + relative_pose.position.y;
    target_pose.pose.position.z =
        first_current_pose.pose.position.z + relative_pose.position.z;

    tf2::Quaternion current_orientation(first_current_pose.pose.orientation.x,
                                        first_current_pose.pose.orientation.y,
                                        first_current_pose.pose.orientation.z,
                                        first_current_pose.pose.orientation.w);
    current_orientation = current_orientation * relative_orientation;
    current_orientation.normalize();

    target_pose.pose.orientation.x = current_orientation.x();
    target_pose.pose.orientation.y = current_orientation.y();
    target_pose.pose.orientation.z = current_orientation.z();
    target_pose.pose.orientation.w = current_orientation.w();


  }
}

/*
    一旦servo_node 进入急停 ，pose_target_cmds就需要停止发送
    从奇异恢复后，等待5次非急停状态计数， 再重新发送 pose_target_cmds 
    因为没有暂停servo_node服务（响应延迟太高）
*/
int status_counter = 0;
void servoStatusCallback(const moveit_msgs::msg::ServoStatus::SharedPtr msg)
    {
        if (msg->code == moveit_msgs::msg::ServoStatus::HALT_FOR_SINGULARITY)
        {
            if (is_servo_halt == false)
            {
              RCLCPP_WARN(rclcpp::get_logger("pose_tracking_right_arm"), "Servo emergency stop detected! Recovering...");
              is_servo_halt = true;

              // 将 servo 模式切换到 关节角度控制模式
              
            }

        }
        else
        {
            if (is_servo_halt == true)
            {
              status_counter++;
              if (status_counter >= 5)
              {
                  RCLCPP_WARN(rclcpp::get_logger("pose_tracking_right_arm"), "Servo Recovering...");
                  status_counter = 0;
                  is_servo_halt = false;
              }
            }

        }
    }



int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node =
      std::make_shared<rclcpp::Node>("pose_tracking_right_arm");

  auto pose_publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/servo_node_right_arm/pose_target_cmds", rclcpp::SystemDefaultsQoS());

  auto switch_input_client =
      node->create_client<moveit_msgs::srv::ServoCommandType>(
          "/servo_node_right_arm/switch_command_type");


  auto vr_pose_subscriber = node->create_subscription<geometry_msgs::msg::Pose>(
      "/arm_right/ee_status", 10, vrPoseCallback);

  //  订阅 Servo 状态  如果Servo进入状态2急停 需要停止发送 pose_target_cmds 直到跳出2
  auto status_sub = node->create_subscription<moveit_msgs::msg::ServoStatus>(
      "/servo_node_right_arm/status", 1, servoStatusCallback);

  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);

  // Spin the node.
  std::thread executor_thread([&executor]() { executor->spin(); });

  // 获取 末端关节 的位姿信息
  try {
    std::this_thread::sleep_for(std::chrono::seconds(15));
    geometry_msgs::msg::TransformStamped transform_stamped =
        tf_buffer.lookupTransform("Body_Link5", "Right_Arm_Link8",
                                  tf2::TimePointZero);
    //first_current_pose.header.frame_id = "Body_Link5";
    first_current_pose.header.frame_id = "Body_Link5";
    first_current_pose.pose.position.x =
        transform_stamped.transform.translation.x;
    first_current_pose.pose.position.y =
        transform_stamped.transform.translation.y;
    first_current_pose.pose.position.z =
        transform_stamped.transform.translation.z;
    first_current_pose.pose.orientation = transform_stamped.transform.rotation;
    RCLCPP_INFO_STREAM(
        node->get_logger(),
        "End-Effector Pose: "
            << "Position: [" << transform_stamped.transform.translation.x
            << ", " << transform_stamped.transform.translation.y << ", "
            << transform_stamped.transform.translation.z << "] "
            << "Orientation: [" << transform_stamped.transform.rotation.w
            << ", " << transform_stamped.transform.rotation.x << ", "
            << transform_stamped.transform.rotation.y << ", "
            << transform_stamped.transform.rotation.z << "]");
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR_STREAM(
        node->get_logger(),
        "Could not transform Body_Link5 to Right_Arm_Link8: " << ex.what());
    rclcpp::shutdown();
    return -1;
  }
  target_pose = first_current_pose;

  // 切换到POSE Servo模式
  auto request =
      std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
  request->command_type = moveit_msgs::srv::ServoCommandType::Request::POSE;
  auto response_future = switch_input_client->async_send_request(request);
  if (response_future.wait_for(std::chrono::duration<double>(3.0)) ==
      std::future_status::timeout) {
    RCLCPP_ERROR_STREAM(
        node->get_logger(),
        "Timed out waiting for MoveIt servo command switching request.");
  }
  const auto response = response_future.get();
  if (response.get()->success) {
    RCLCPP_INFO_STREAM(node->get_logger(),
                       "Switched to command input type: Pose");
  } else {
    RCLCPP_ERROR_STREAM(node->get_logger(),
                        "Could not switch MoveIt servo command input type.");
  }

  // Follow the trajectory
  rclcpp::Rate rate(100);
  geometry_msgs::msg::PoseStamped last_pose;  // 存储上一次的目标姿态
  while (rclcpp::ok())
  {
    // 如果 target_pose 没有变化，则跳过发布
    if (is_servo_halt == false && ( target_pose.pose.position.x != last_pose.pose.position.x ||
        target_pose.pose.position.y != last_pose.pose.position.y ||
        target_pose.pose.position.z != last_pose.pose.position.z ||
        target_pose.pose.orientation.x != last_pose.pose.orientation.x ||
        target_pose.pose.orientation.y != last_pose.pose.orientation.y ||
        target_pose.pose.orientation.z != last_pose.pose.orientation.z ||
        target_pose.pose.orientation.w != last_pose.pose.orientation.w))
    {
      target_pose.header.stamp = node->now();
      pose_publisher->publish(target_pose);
      last_pose = target_pose;  // 更新 last_pose
    }

    rate.sleep();
  }

  executor->cancel();
  if (executor_thread.joinable()) {
    executor_thread.join();
  }
  rclcpp::shutdown();
}
