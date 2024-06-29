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

#ifndef TF2_BROADCASTER__TF2_BROADCASTER_BASE_HPP_
#define TF2_BROADCASTER__TF2_BROADCASTER_BASE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_broadcaster/visibility_control.h"

#include "realtime_tools/realtime_publisher.h"
#include "tf2_msgs/msg/tf_message.hpp"

namespace tf2_broadcaster
{
  using RefType = std_msgs::msg::Float64MultiArray;

  /**
   * \brief broadcasting transformed states from sensor
   *
   */
  class Tf2BroadcasterBase : public controller_interface::ControllerInterface
  {
  public:
    TF2_BROADCASTER_PUBLIC
    Tf2BroadcasterBase();

    TF2_BROADCASTER_PUBLIC
    ~Tf2BroadcasterBase() = default;

    TF2_BROADCASTER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    TF2_BROADCASTER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    TF2_BROADCASTER_PUBLIC
    controller_interface::CallbackReturn on_init() override;

    TF2_BROADCASTER_PUBLIC
    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    TF2_BROADCASTER_PUBLIC
    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    TF2_BROADCASTER_PUBLIC
    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    TF2_BROADCASTER_PUBLIC
    controller_interface::return_type update(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  protected:
    virtual void declare_parameters() = 0;

    virtual controller_interface::CallbackReturn read_parameters() = 0;

    std::string parent_frame_id;
    std::string child_frame_id;
    
    std::vector<std::string> state_interface_types_;

    std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> odometry_transform_publisher_;
    std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
        realtime_odometry_transform_publisher_;
  };

} // namespace tf2_broadcaster

#endif // TF2_BROADCASTER__TF2_BROADCASTER_BASE_HPP_
