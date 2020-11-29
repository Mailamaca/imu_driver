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

#ifndef COMPOSITION__IMU_DRIVER_COMPONENT_HPP_
#define COMPOSITION__IMU_DRIVER_COMPONENT_HPP_

#include "imu_driver/visibility_control.h"

#include "i2c_interfaces/srv/i2c_command.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "rclcpp/rclcpp.hpp"

#include <string>


#define ESP32_IMU_REG 0x20
#define ESP32_IMU_LENGTH 12

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace imu_driver
{

class ImuDriverComponent : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit ImuDriverComponent(const rclcpp::NodeOptions & options);

private:
  rclcpp::TimerBase::SharedPtr timer_command;
  void timer_command_callback();
  void start_timer_command();
  
  rclcpp::Client<i2c_interfaces::srv::I2cCommand>::SharedPtr i2c_command;
  bool call_i2c_command_service(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Request> request, std::function<void(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response>)> fun);
  bool wait_for_i2c_command_service();

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_info;
  
  void create_subscriptions();
  void create_publishers();
  void create_clients();

  void load_parameters();
  void initialize();
  
  void read_cb(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response> response);

  
  std::string imu_topic_name;    // = "/hw/imu_raw";
    
  int out_msgs_period;
  int i2c_slave_address;  
  
  union unionfloat{
    float i;
    uint8_t c[4];
  };
};

}  // namespace encoders

#endif  // COMPOSITION__IMU_DRIVER_COMPONENT_HPP_
