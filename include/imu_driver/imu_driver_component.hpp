// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef COMPOSITION__ENCODERS_COMPONENT_HPP_
#define COMPOSITION__ENCODERS_COMPONENT_HPP_

#include "encoders/visibility_control.h"

#include "i2c_interfaces/srv/i2c_command.hpp"
#include "maila_msgs/msg/encoder.hpp"
#include "maila_msgs/msg/encoder_array.hpp"

#include "rclcpp/rclcpp.hpp"

#include <string>


#define ESP32_ENC_REG 0x10
#define ESP32_ENC_NUMS 5

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace encoders
{

class EncodersComponent : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit EncodersComponent(const rclcpp::NodeOptions & options);

private:
  rclcpp::TimerBase::SharedPtr timer_command;
  void timer_command_callback();
  void start_timer_command();
  
  rclcpp::Client<i2c_interfaces::srv::I2cCommand>::SharedPtr i2c_command;
  bool call_i2c_command_service(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Request> request, std::function<void(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response>)> fun);
  bool wait_for_i2c_command_service();

  rclcpp::Publisher<maila_msgs::msg::EncoderArray>::SharedPtr pub_info;
  
  void create_subscriptions();
  void create_publishers();
  void create_clients();

  void load_parameters();
  void initialize();
  
  void read_cb(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response> response);
  maila_msgs::msg::Encoder encoderCalculation(int idx, uint16_t tick, uint32_t micros);

  
  std::string encoders_topic_name;    // = "/hw/encoders";
    
  int out_msgs_period;
  int i2c_slave_address;
  
  uint8_t ids[ESP32_ENC_NUMS];
  float scale_factors[ESP32_ENC_NUMS];   
  float distances[ESP32_ENC_NUMS];
  
  
};

}  // namespace encoders

#endif  // COMPOSITION__ENCODERS_COMPONENT_HPP_
