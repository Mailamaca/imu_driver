#include "imu_driver/imu_driver_component.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <atomic>


namespace imu_driver
{

ImuDriverComponent::ImuDriverComponent(const rclcpp::NodeOptions & options)
: Node("ImuDriverComponent", options) {
  RCLCPP_INFO( this->get_logger(), "ImuDriverComponent::ImuDriverComponent");

  load_parameters();    
  
  create_publishers();
  create_subscriptions();
  create_clients();    

  initialize();  
}


void ImuDriverComponent::load_parameters() {

  this->declare_parameter<std::string>("topics.out_imu", "a");
  this->get_parameter("topics.out_imu", imu_topic_name);
  //RCLCPP_INFO( this->get_logger(), "topics.out_imu: %s", imu_topic_name.c_str());

  this->declare_parameter<int>("out_msgs_period", 10000);
  this->get_parameter("out_msgs_period", out_msgs_period);
  
  this->declare_parameter<int>("i2c_slave_address", 10000);
  this->get_parameter("i2c_slave_address", i2c_slave_address);

}

void ImuDriverComponent::create_subscriptions() {
  //subs_mode = this->create_subscription<maila_msgs::msg::VehicleMode>(
  //  mode_topic_name, 10, std::bind(&MotorComponent::subs_mode_callback, this, _1));
  //this->start_timer_subs_mode_timeout();

}

void ImuDriverComponent::create_publishers() {
  pub_info = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_name, 10);
  
}

void ImuDriverComponent::create_clients() {
  i2c_command = this->create_client<i2c_interfaces::srv::I2cCommand>("i2c_command");

}

void ImuDriverComponent::initialize() {
    
  this->start_timer_command();

}

bool ImuDriverComponent::call_i2c_command_service(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Request> request, std::function<void(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response>)> cb) {

  //RCLCPP_INFO( this->get_logger(), "call_i2c_command_service");

  if (!i2c_command->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Interrupted while waiting for 'i2c_command' service. Exiting.");
      return false;
    }
    RCLCPP_WARN(
      this->get_logger(),
      "Service 'i2c_command' not available after waiting");
    return false;
  }

  using ServiceResponseFuture =
    rclcpp::Client<i2c_interfaces::srv::I2cCommand>::SharedFuture;
  auto response_received_callback = [/*this, */cb](ServiceResponseFuture future) {
    //RCLCPP_INFO( this->get_logger(), "response_received_callback");
    cb(future.get());
  };

  //RCLCPP_INFO( this->get_logger(), "async_send_request");
  auto future_result = i2c_command->async_send_request(request, response_received_callback);
  
  return true;
}

void ImuDriverComponent::read_cb(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response> response) {
  
  //RCLCPP_INFO( this->get_logger(), "pca9685_cmd_cb");

  // evaluate response
  if (!response->ok) {
    RCLCPP_ERROR( this->get_logger(), "Error on read from i2c");
    return;
  }
  
  if (response->data_received.size() != (ESP32_IMU_LENGTH)) {
    RCLCPP_ERROR( this->get_logger(), "Error on read from i2c, size %d diff from %d", response->data_received.size(),ESP32_IMU_LENGTH);
    return;
  }  
  
  int idx = 0;
  ImuDriverComponent::unionfloat aX;
  RCLCPP_INFO( this->get_logger(),
               "0x%02x 0x%02x 0x%02x 0x%02x ",
               response->data_received.at(0),
               response->data_received.at(1),
               response->data_received.at(2),
               response->data_received.at(3));

  aX.c[0] = response->data_received.at(idx+0);
  aX.c[1] = response->data_received.at(idx+1);
  aX.c[2] = response->data_received.at(idx+2);
  aX.c[3] = response->data_received.at(idx+3);  
  idx += 4;
  ImuDriverComponent::unionfloat aY;
  aY.c[0] = response->data_received.at(idx+0);
  aY.c[1] = response->data_received.at(idx+1);
  aY.c[2] = response->data_received.at(idx+2);
  aY.c[3] = response->data_received.at(idx+3);  
  idx += 4;
  ImuDriverComponent::unionfloat aZ;
  aZ.c[0] = response->data_received.at(idx+0);
  aZ.c[1] = response->data_received.at(idx+1);
  aZ.c[2] = response->data_received.at(idx+2);
  aZ.c[3] = response->data_received.at(idx+3);  
  idx += 4;
  
  
  geometry_msgs::msg::Vector3 linear_acceleration;
  linear_acceleration.x = aX.i;
  linear_acceleration.y = aY.i;
  linear_acceleration.z = aZ.i;


  sensor_msgs::msg::Imu imu_msg;
  imu_msg.linear_acceleration = linear_acceleration;
  pub_info->publish(imu_msg);
 
  
}

void ImuDriverComponent::start_timer_command() {
  std::chrono::duration<int, std::milli> timeout_time(out_msgs_period);
  this->timer_command = this->create_wall_timer(
    timeout_time, std::bind(&ImuDriverComponent::timer_command_callback, this)
    );
}

void ImuDriverComponent::timer_command_callback() {
  
  auto request = std::make_shared<i2c_interfaces::srv::I2cCommand::Request>();
  request->slave = i2c_slave_address;
  request->reg = ESP32_IMU_REG;
  request->write = false;
  request->length = ESP32_IMU_LENGTH;
  std::vector<uint8_t> vectData;
  request->data_to_send = vectData;

  auto fp = std::bind(&ImuDriverComponent::read_cb, this, _1);
  call_i2c_command_service(request, fp); 

}

}  // namespace encoders

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(imu_driver::ImuDriverComponent)

