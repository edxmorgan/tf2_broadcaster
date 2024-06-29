// Copyright 2024 Edward Morgan
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

#include "tf2_broadcaster/tf2_broadcaster.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace tf2_broadcaster
{
  Tf2Broadcaster::Tf2Broadcaster() : Tf2BroadcasterBase() {}

  void Tf2Broadcaster::declare_parameters()
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  }

  controller_interface::CallbackReturn Tf2Broadcaster::read_parameters()
  {
    if (!param_listener_)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
      return controller_interface::CallbackReturn::ERROR;
    };
    params_ = param_listener_->get_params();

    if (params_.sensor.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "'sensor' parameter was empty");
      return controller_interface::CallbackReturn::ERROR;
    };
    RCLCPP_INFO(get_node()->get_logger(), "Sensor Registered --> %s ", params_.sensor.c_str());

    if (params_.child_frame_id.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "'child_frame_id' parameter was empty");
      return controller_interface::CallbackReturn::ERROR;
    };
    RCLCPP_INFO(get_node()->get_logger(), "child_frame_id Registered --> %s ", params_.child_frame_id.c_str());
    child_frame_id = params_.child_frame_id;

    if (params_.parent_frame_id.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "'parent_frame_id' parameter was empty");
      return controller_interface::CallbackReturn::ERROR;
    };
    RCLCPP_INFO(get_node()->get_logger(), "parent_frame_id Registered --> %s ", params_.parent_frame_id.c_str());
    parent_frame_id = params_.parent_frame_id;

    if (params_.position_x_state_interface.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "'position_x_state_interface' parameter was empty");
      return controller_interface::CallbackReturn::ERROR;
    };
    RCLCPP_INFO(get_node()->get_logger(), "position_x_state_interface Registered --> %s ", params_.position_x_state_interface.c_str());
    state_interface_types_.push_back(params_.sensor + "/" + params_.position_x_state_interface);

    if (params_.position_y_state_interface.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "'position_y_state_interface' parameter was empty");
      return controller_interface::CallbackReturn::ERROR;
    };
    RCLCPP_INFO(get_node()->get_logger(), "position_y_state_interface Registered --> %s ", params_.position_y_state_interface.c_str());
    state_interface_types_.push_back(params_.sensor + "/" + params_.position_y_state_interface);

    if (params_.position_z_state_interface.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "'position_z_state_interface' parameter was empty");
      return controller_interface::CallbackReturn::ERROR;
    };
    RCLCPP_INFO(get_node()->get_logger(), "position_z_state_interface Registered --> %s ", params_.position_z_state_interface.c_str());
    state_interface_types_.push_back(params_.sensor + "/" + params_.position_z_state_interface);

    if (params_.orientation_x_state_interface.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "'orientation_x_state_interface' parameter was empty");
      return controller_interface::CallbackReturn::ERROR;
    };
    RCLCPP_INFO(get_node()->get_logger(), "orientation_x_state_interface Registered --> %s ", params_.orientation_x_state_interface.c_str());
    state_interface_types_.push_back(params_.sensor + "/" + params_.orientation_x_state_interface);

    if (params_.orientation_y_state_interface.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "'orientation_y_state_interface' parameter was empty");
      return controller_interface::CallbackReturn::ERROR;
    };
    RCLCPP_INFO(get_node()->get_logger(), "orientation_y_state_interface Registered --> %s ", params_.orientation_y_state_interface.c_str());
    state_interface_types_.push_back(params_.sensor + "/" + params_.orientation_y_state_interface);

    if (params_.orientation_z_state_interface.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "'orientation_z_state_interface' parameter was empty");
      return controller_interface::CallbackReturn::ERROR;
    };
    RCLCPP_INFO(get_node()->get_logger(), "orientation_z_state_interface Registered --> %s ", params_.orientation_z_state_interface.c_str());
    state_interface_types_.push_back(params_.sensor + "/" + params_.orientation_z_state_interface);

    if (params_.orientation_w_state_interface.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "'orientation_w_state_interface' parameter was empty");
      return controller_interface::CallbackReturn::ERROR;
    };
    RCLCPP_INFO(get_node()->get_logger(), "orientation_w_state_interface Registered --> %s ", params_.orientation_w_state_interface.c_str());
    state_interface_types_.push_back(params_.sensor + "/" + params_.orientation_w_state_interface);


    return controller_interface::CallbackReturn::SUCCESS;
  }

} // namespace tf2_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    tf2_broadcaster::Tf2Broadcaster, controller_interface::ControllerInterface)
