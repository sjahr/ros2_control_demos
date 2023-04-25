// Copyright 2021 Stogl Robotics Consulting UG (haftungsbescrhänkt)
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

#include "joint_mode_controller/joint_mode_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace joint_mode_controller
{
JointModeController::JointModeController() : controller_interface::ControllerInterface()
/* TODO(sjahr): Re-enable once an interface will be created,
  rt_command_ptr_(nullptr)*/
{
}

controller_interface::CallbackReturn JointModeController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointModeController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  if (params_.joints.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.interface_name.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'interface_name' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : params_.joints)
  {
    command_interface_types_.push_back(joint + "/" + params_.interface_name);
  }

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration JointModeController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_types_;

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration JointModeController::state_interface_configuration()
  const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn JointModeController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    ordered_interfaces;
  if (
    !controller_interface::get_ordered_interfaces(
      command_interfaces_, command_interface_types_, std::string(""), ordered_interfaces) ||
    command_interface_types_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
      command_interface_types_.size(), ordered_interfaces.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // TODO(sjahr): Re-enable once an interface will be created
  // reset command buffer if a command came through callback when controller was inactive
  //rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

  next_mode_ += 1.0;

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointModeController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset command buffer
  //rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type JointModeController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // auto joint_mode_commands = rt_command_ptr_.readFromRT();

  // no command received yet
  // if (!joint_mode_commands || !(*joint_mode_commands))
  // {
  //   return controller_interface::return_type::OK;
  // }

  for (auto & interface : command_interfaces_)
  {
    interface.set_value(next_mode_);
  }

  return controller_interface::return_type::OK;
}

}  // namespace joint_mode_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_mode_controller::JointModeController, controller_interface::ControllerInterface)
