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

#include "ymbot_d_ros2_controller_sharedmemory/SharedMemoryArm.hpp"

#include "ymbot_d_ros2_controller_sharedmemory/ymbot_d_hardware_interface.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <rclcpp/utilities.hpp>

using namespace std;

namespace ymbot_d_ros2_controller_sharedmemory {

int n_motors = JOINT_ARM_NUMBER;
int n_joints;

SharedMemoryArm shmArm(false);
EuArmData armRes_[JOINT_ARM_NUMBER];
double expected_pos[JOINT_ARM_NUMBER];

const char *green = "\033[1;32m";
const char *red = "\033[1;31m";
const char *reset = "\033[0m";

hardware_interface::CallbackReturn ymbot_d_hardware_interface::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // get paramaters from config file

  // get paramaters from urdf file
  n_joints = info_.joints.size();
  hw_positions_.resize(n_joints, std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(n_joints, std::numeric_limits<double>::quiet_NaN());
  RCLCPP_INFO(rclcpp::get_logger("ymbot_d_hardware_interface"),
              "%sNumber of joints: %d%s", green, n_joints, reset);

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    // DiffBotSystem has exactly one states and one command interface on each
    // joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("ymbot_d_hardware_interface"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name !=
        hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          rclcpp::get_logger("ymbot_d_hardware_interface"),
          "Joint '%s' have %s command interfaces found. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("ymbot_d_hardware_interface"),
                   "Joint '%s' has %zu state interface. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          rclcpp::get_logger("ymbot_d_hardware_interface"),
          "Joint '%s' have '%s' as first state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ymbot_d_hardware_interface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (int i = 0; i < n_joints; i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_positions_[i]));
  }
  RCLCPP_INFO(rclcpp::get_logger("ymbot_d_hardware_interface"),
              "%sexport_state_interfaces successfully%s", green, reset);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ymbot_d_hardware_interface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (int i = 0; i < n_joints; i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_commands_[i]));
  }
  RCLCPP_INFO(rclcpp::get_logger("ymbot_d_hardware_interface"),
              "%sexport_command_interfaces successfully%s", green, reset);

  return command_interfaces;
}

hardware_interface::CallbackReturn ymbot_d_hardware_interface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("ymbot_d_hardware_interface"),
              "%stest on_configure%s", green, reset);

  for (int i = 0; i < n_joints; i++) {
    RCLCPP_INFO(rclcpp::get_logger("ymbot_d_hardware_interface"),
                "%sjoint name: %s%s", green, info_.joints[i].name.c_str(),
                reset);
  }

  RCLCPP_INFO(rclcpp::get_logger("ymbot_d_hardware_interface"),
              "%sAll robot joints: %d and all controlled motors: %d%s", green,
              n_joints, n_motors, reset);

  if ((n_motors != n_joints)) {
    RCLCPP_ERROR(rclcpp::get_logger("ymbot_d_hardware_interface"),
                 "%sthe number of 'motors id' is incorrect%s", red, reset);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // if (static_cast<int>(joints_offset_angle.size()) != n_motors) {
  //   RCLCPP_ERROR(rclcpp::get_logger("ymbot_d_hardware_interface"),
  //                "%sthe number of 'joints offset' is incorrect%s", red, reset);
  //   return hardware_interface::CallbackReturn::ERROR;
  // }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ymbot_d_hardware_interface::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("ymbot_d_hardware_interface"),
              "%stest on_cleanup%s", green, reset);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ymbot_d_hardware_interface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("ymbot_d_hardware_interface"),
              "%sActivating ...please wait...%s", green, reset);
  // for (int i = 0; i < JOINT_ARM_NUMBER; i++)
  // {
  //   RCLCPP_INFO(rclcpp::get_logger("ymbot_d_hardware_interface"),
  //             "%sActivating ... %splease wait...%s", green, info_.joints[i].name.c_str(), reset);
  // }
  
    
  // 读取第一次共享内存更新的数据 by chen furong  2025.5.16
  while (true)
  {
    try {
      shmArm.readJointDatafromMotorArm(armRes_);
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (armRes_[0].pos_ == 0){
      // std::cout << "[Shared memory] has not updated motor data, check eumotor driver..." << std::endl;
      RCLCPP_INFO(rclcpp::get_logger("ymbot_d_hardware_interface"), 
            "[Shared memory] has not updated motor data, check eumotor driver..."
            );
      continue;
    }
    else{
      break;
    }
  }
  for (int i = 0; i < JOINT_ARM_NUMBER; i++)
  { 
    // RCLCPP_INFO(rclcpp::get_logger("ymbot_d_hardware_interface"), 
    //             "read: Motor index: %d; current joint pos: %.6f du; vel: %.6f rad/s; current: %.6f A",
    //             i, armRes_[i].pos_*180.0/M_PI, armRes_[i].vel_, armRes_[i].cur_);
    hw_positions_[i] = armRes_[i].pos_;  // 按config里的顺序
    
  }

  // 更换21和22的电机顺序 by chenfurong 2025.5.23
  // double motor21_pos = hw_positions_[4];
  // hw_positions_[4] = hw_positions_[5];
  // hw_positions_[5] = motor21_pos;

  for (int i = 0; i < JOINT_ARM_NUMBER; i++)
  { 
    hw_commands_[i] = hw_positions_[i];
    // RCLCPP_INFO(rclcpp::get_logger("ymbot_d_hardware_interface"), 
    //         "read: Motor index: %d; current joint pos: %.6f du",
    //         i, hw_commands_[i]*180.0/M_PI);
  }

  RCLCPP_INFO(rclcpp::get_logger("ymbot_d_hardware_interface"),
              "%sSuccessfully activated!%s", green, reset);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ymbot_d_hardware_interface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("ymbot_d_hardware_interface"),
              "%sDeactivating ...please wait...%s", green, reset);
  shmArm.~SharedMemoryArm();

  RCLCPP_INFO(rclcpp::get_logger("ymbot_d_hardware_interface"),
              "%sSuccessfully deactivated!%s", green, reset);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
ymbot_d_hardware_interface::read(const rclcpp::Time & /*time*/,
                                 const rclcpp::Duration &period) {

  // 从共享内存中读取电机数据 by chen furong 2025.5.16
  try {
    shmArm.readJointDatafromMotorArm(armRes_);
  } catch (const std::exception& e) {
      // std::cerr << e.what() << std::endl;
      RCLCPP_ERROR(rclcpp::get_logger("ymbot_d_hardware_interface"),
                   "[sharedmemory] fail to read motor data from shared memory!");
      return hardware_interface::return_type::ERROR;
  }
  for (int i = 0; i < JOINT_ARM_NUMBER; i++)
  { 
    // if (i < 4)
    // {
    //   RCLCPP_INFO(rclcpp::get_logger("ymbot_d_hardware_interface"), 
    //             "read: Motor index: %d; current joint pos: %.6f du; vel: %.6f rad/s; current: %.6f A",
    //             i, armRes_[i].pos_*180.0/M_PI, armRes_[i].vel_, armRes_[i].cur_);
    // }
    
    hw_positions_[i] = armRes_[i].pos_;  // 按config里的顺序
  }

  // 更换21和22的电机顺序 by chenfurong 2025.5.23
  // double motor21_pos = hw_positions_[4];
  // hw_positions_[4] = hw_positions_[5];
  // hw_positions_[5] = motor21_pos;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
ymbot_d_hardware_interface::write(const rclcpp::Time & /*time*/,
                                  const rclcpp::Duration &period) {
  // for (int i = 0; i < JOINT_ARM_NUMBER; i++)
  // {
  //   RCLCPP_INFO(rclcpp::get_logger("ymbot_d_hardware_interface"),
  //             "%sActivating ... %splease wait...%s", green, info_.joints[i].name.c_str(), reset);
  // }

  // 目标关节角度，写入共享内存。
  // 更换21和22的电机顺序 by chenfurong 2025.5.23
  // double motor22_targetpos = hw_commands_[4];
  // hw_commands_[4] = hw_commands_[5];
  // hw_commands_[5] = motor22_targetpos;

  if (hw_commands_.size() != 20) {
    RCLCPP_ERROR(rclcpp::get_logger("ymbot_d_hardware_interface"),
                 "%sthe number of 'hw_commands_' is incorrect%s", red, reset);
    return hardware_interface::return_type::ERROR;
  }
  for (size_t i = 0; i < hw_commands_.size(); i++)
  { 
    expected_pos[i] = hw_commands_[i];
    // RCLCPP_INFO(rclcpp::get_logger("ymbot_d_hardware_interface"), 
    //             "write: Motor index: %ld; target joint pos: %.6f du",
    //             i, hw_commands_[i]*180.0/M_PI);
  }
  try {
    shmArm.writeJointDatatoMotorArm(expected_pos);
  } catch (const std::exception& e) {
    // std::cerr << e.what() << std::endl;
    RCLCPP_ERROR(rclcpp::get_logger("ymbot_d_hardware_interface"),
                 "[sharedmemory] fail to write motor data to shared memory!");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

} // namespace ymbot_d_ros2_controller_sharedmemory

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ymbot_d_ros2_controller_sharedmemory::ymbot_d_hardware_interface,
                       hardware_interface::SystemInterface)
