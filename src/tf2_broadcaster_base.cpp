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

#include "tf2_broadcaster/tf2_broadcaster_base.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace
{
  constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
} // namespace

namespace tf2_broadcaster
{
  Tf2BroadcasterBase::Tf2BroadcasterBase()
      : controller_interface::ControllerInterface(),
        odometry_transform_publisher_(nullptr),
        realtime_odometry_transform_publisher_(nullptr)
  {
  }

  controller_interface::CallbackReturn Tf2BroadcasterBase::on_init()
  {
    try
    {
      declare_parameters();
    }
    catch (const std::exception &e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn Tf2BroadcasterBase::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    auto ret = this->read_parameters();
    if (ret != controller_interface::CallbackReturn::SUCCESS)
    {
      return ret;
    }

    // initialize transform publisher and message
    try
    {
      odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
          DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());

      realtime_odometry_transform_publisher_ =
          std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
              odometry_transform_publisher_);

      auto &odometry_transform_message = realtime_odometry_transform_publisher_->msg_;
      odometry_transform_message.transforms.resize(1);
      odometry_transform_message.transforms.front().header.frame_id = parent_frame_id;
      odometry_transform_message.transforms.front().child_frame_id = child_frame_id;
    }
    catch (const std::exception &e)
    {
      fprintf(
          stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
          e.what());
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration Tf2BroadcasterBase::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration Tf2BroadcasterBase::state_interface_configuration()
      const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    state_interfaces_config.names = state_interface_types_;

    return state_interfaces_config;
  }

  controller_interface::CallbackReturn Tf2BroadcasterBase::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        ordered_interfaces;
    if (
        !controller_interface::get_ordered_interfaces(
            state_interfaces_, state_interface_types_, std::string(""), ordered_interfaces) ||
        state_interface_types_.size() != ordered_interfaces.size())
    {
      RCLCPP_ERROR(
          get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
          state_interface_types_.size(), ordered_interfaces.size());
      return controller_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), "activate successful");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn Tf2BroadcasterBase::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type Tf2BroadcasterBase::update(
      const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
  {
    if (realtime_odometry_transform_publisher_->trylock())
    {
      auto &transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
      transform.header.stamp = time;
      transform.transform.translation.x = state_interfaces_[0].get_value();
      transform.transform.translation.y = state_interfaces_[1].get_value();
      transform.transform.translation.z = state_interfaces_[2].get_value();
      transform.transform.rotation.x = state_interfaces_[3].get_value();
      transform.transform.rotation.y = state_interfaces_[4].get_value();
      transform.transform.rotation.z = state_interfaces_[5].get_value();
      transform.transform.rotation.w = state_interfaces_[6].get_value();
      realtime_odometry_transform_publisher_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
  }

} // namespace tf2_broadcaster
