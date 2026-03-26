// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef YMBOT_D_ROS2_CONTROLLER_SHAREDMEMORY__DIFFBOT_SYSTEM_HPP_
#define YMBOT_D_ROS2_CONTROLLER_SHAREDMEMORY__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "ymbot_d_ros2_controller_sharedmemory/visibility_control.h"

namespace ymbot_d_ros2_controller_sharedmemory {
class ymbot_d_hardware_interface : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ymbot_d_hardware_interface)

  YMBOT_D_ROS2_CONTROLLER_SHAREDMEMORY_PUBLIC
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &info) override;

  YMBOT_D_ROS2_CONTROLLER_SHAREDMEMORY_PUBLIC
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  YMBOT_D_ROS2_CONTROLLER_SHAREDMEMORY_PUBLIC
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  YMBOT_D_ROS2_CONTROLLER_SHAREDMEMORY_PUBLIC
  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  YMBOT_D_ROS2_CONTROLLER_SHAREDMEMORY_PUBLIC
  hardware_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  YMBOT_D_ROS2_CONTROLLER_SHAREDMEMORY_PUBLIC
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  YMBOT_D_ROS2_CONTROLLER_SHAREDMEMORY_PUBLIC
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  YMBOT_D_ROS2_CONTROLLER_SHAREDMEMORY_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  YMBOT_D_ROS2_CONTROLLER_SHAREDMEMORY_PUBLIC
  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
};

} // namespace ymbot_d_ros2_controller_sharedmemory

#endif // YMBOT_D_ROS2_CONTROLLER_SHAREDMEMORY__DIFFBOT_SYSTEM_HPP_
