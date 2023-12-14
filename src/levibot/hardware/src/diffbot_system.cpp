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

#include "levibot/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace levibot
{
hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // Get the joint names & Parameters from the "ros2_control -> <hardware> tag" section of the URDF
  // Store all the information in info_
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  config_.device = info_.hardware_parameters["device_serial"];
  config_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  hw_interfaces_[0].name = info_.hardware_parameters["right_wheel_joint_name"];
  hw_interfaces_[1].name = info_.hardware_parameters["left_wheel_joint_name"];

  // for (const hardware_interface::ComponentInfo & joint : info_.joints)
  // {
  //   // DiffBotSystem has exactly two states and one command interface on each joint
  //   if (joint.command_interfaces.size() != 1)
  //   {
  //     RCLCPP_FATAL(
  //       rclcpp::get_logger("DiffBotSystemHardware"),
  //       "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
  //       joint.command_interfaces.size());
  //     return hardware_interface::CallbackReturn::ERROR;
  //   }

  //   if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
  //   {
  //     RCLCPP_FATAL(
  //       rclcpp::get_logger("DiffBotSystemHardware"),
  //       "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
  //       joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
  //     return hardware_interface::CallbackReturn::ERROR;
  //   }

  //   if (joint.state_interfaces.size() != 2)
  //   {
  //     RCLCPP_FATAL(
  //       rclcpp::get_logger("DiffBotSystemHardware"),
  //       "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
  //       joint.state_interfaces.size());
  //     return hardware_interface::CallbackReturn::ERROR;
  //   }

  //   if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  //   {
  //     RCLCPP_FATAL(
  //       rclcpp::get_logger("DiffBotSystemHardware"),
  //       "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
  //       joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
  //     return hardware_interface::CallbackReturn::ERROR;
  //   }

  //   if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
  //   {
  //     RCLCPP_FATAL(
  //       rclcpp::get_logger("DiffBotSystemHardware"),
  //       "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
  //       joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
  //     return hardware_interface::CallbackReturn::ERROR;
  //   }
  // }

  // If everything is fine, return success
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < 2; i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      hw_interfaces_[i].name, hardware_interface::HW_IF_POSITION, &hw_interfaces_[i].position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      hw_interfaces_[i].name, hardware_interface::HW_IF_VELOCITY, &hw_interfaces_[i].velocity));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < 2; i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      hw_interfaces_[i].name, hardware_interface::HW_IF_VELOCITY, &hw_interfaces_[i].velocity));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Activating ...");
  // bot_comms_.connect();
  if (!bot_comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // set some default values
  for (auto i = 0u; i < 2; i++)
  {
    if (std::isnan(hw_interfaces_[i].position))
    {
      hw_interfaces_[i].velocity = 0;
      hw_interfaces_[i].position = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Deactivating ...");
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{

  for (std::size_t i = 0; i < 2; i++)
  {
    // Simulate DiffBot wheels's movement as a first-order system
    // Update the joint status: this is a revolute joint without any limit.
    // Simply integrates
    hw_interfaces_[i].position = hw_interfaces_[i].position + period.seconds() * hw_interfaces_[i].velocity;
    
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type levibot ::DiffBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Writing...");
  bot_comms_.set_motor_values(hw_interfaces_[0].velocity, hw_interfaces_[1].velocity);
  RCLCPP_INFO(rclcpp::get_logger("DiffBotSystemHardware"), "Joints successfully written!");

  return hardware_interface::return_type::OK;
}

}  // namespace levibot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  levibot::DiffBotSystemHardware, hardware_interface::SystemInterface)
