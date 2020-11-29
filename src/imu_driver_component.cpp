#include "encoders/encoders_component.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <atomic>


namespace encoders
{

EncodersComponent::EncodersComponent(const rclcpp::NodeOptions & options)
: Node("Encoders", options) {
  RCLCPP_INFO( this->get_logger(), "EncodersComponent::EncodersComponent");

  load_parameters();    
  
  create_publishers();
  create_subscriptions();
  create_clients();    

  initialize();  
}


void EncodersComponent::load_parameters() {

  this->declare_parameter<std::string>("topics.out_encoders", "a");
  this->get_parameter("topics.out_encoders", encoders_topic_name);
  //RCLCPP_INFO( this->get_logger(), "topics.out_encoders: %s", encoders_topic_name.c_str());

  this->declare_parameter<int>("out_msgs_period", 10000);
  this->get_parameter("out_msgs_period", out_msgs_period);
  
  this->declare_parameter<int>("i2c_slave_address", 10000);
  this->get_parameter("i2c_slave_address", i2c_slave_address);

  for (int i=0; i < ESP32_ENC_NUMS; i++) {
  
    std::string str_id = "enc" + std::to_string(i) + ".id";
    this->declare_parameter<uint8_t>(str_id, 1);
    this->get_parameter(str_id, ids[i]);
    
    std::string str_sf = "enc" + std::to_string(i) + ".scale_factor";
    this->declare_parameter<float>(str_sf, 1);
    this->get_parameter(str_sf, scale_factors[i]);
  }

}

void EncodersComponent::create_subscriptions() {
  //subs_mode = this->create_subscription<maila_msgs::msg::VehicleMode>(
  //  mode_topic_name, 10, std::bind(&MotorComponent::subs_mode_callback, this, _1));
  //this->start_timer_subs_mode_timeout();

}

void EncodersComponent::create_publishers() {
  pub_info = this->create_publisher<maila_msgs::msg::EncoderArray>(encoders_topic_name, 10);
  
}

void EncodersComponent::create_clients() {
  i2c_command = this->create_client<i2c_interfaces::srv::I2cCommand>("i2c_command");

}

void EncodersComponent::initialize() {
    
  this->start_timer_command();

}

bool EncodersComponent::call_i2c_command_service(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Request> request, std::function<void(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response>)> cb) {

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

void EncodersComponent::read_cb(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response> response) {
  
  //RCLCPP_INFO( this->get_logger(), "pca9685_cmd_cb");

  // evaluate response
  if (!response->ok) {
    RCLCPP_ERROR( this->get_logger(), "Error on read from i2c");
    return;
  }
  
  if (response->data_received.size() != (4+2*ESP32_ENC_NUMS)) {
    RCLCPP_ERROR( this->get_logger(), "Error on read from i2c, size diff from 14");
    return;
  }  
  
  int idx = 0;
  uint32_t micros = ((uint32_t)response->data_received.at(idx++) << 24) |
                    ((uint32_t)response->data_received.at(idx++) << 16) |
                    ((uint32_t)response->data_received.at(idx++) <<  8) |
                    ((uint32_t)response->data_received.at(idx++) <<  0);
        
  std::vector<maila_msgs::msg::Encoder> encoders_vect;
  for (int i =0; i < ESP32_ENC_NUMS; i++) {
    uint16_t tk_enc = ((uint16_t)response->data_received.at(idx++) << 8) |
                      ((uint16_t)response->data_received.at(idx++) << 0);
    encoders_vect.push_back(encoderCalculation(i, tk_enc, micros));  
  }
  
  maila_msgs::msg::EncoderArray encoders_msg;
  encoders_msg.encoders = encoders_vect;
  pub_info->publish(encoders_msg);
  
  
}

maila_msgs::msg::Encoder EncodersComponent::encoderCalculation(int idx, uint16_t tick, uint32_t micros) {

  this->distances[idx] += (float)tick * scale_factors[idx];
  
  float speed = (float)tick * scale_factors[idx] / ((float)micros * 1000000); 
  
  float speed_cov = 1;     

  maila_msgs::msg::Encoder msg;
  msg.id = ids[idx];
  msg.distance = this->distances[idx];
  msg.speed = speed;
  msg.speed_covariance = speed_cov;
  
  return msg;
}

void EncodersComponent::start_timer_command() {
  std::chrono::duration<int, std::milli> timeout_time(out_msgs_period);
  this->timer_command = this->create_wall_timer(
    timeout_time, std::bind(&EncodersComponent::timer_command_callback, this)
    );
}

void EncodersComponent::timer_command_callback() {
  
  auto request = std::make_shared<i2c_interfaces::srv::I2cCommand::Request>();
  request->slave = i2c_slave_address;
  request->reg = ESP32_ENC_REG;
  request->write = false;
  request->length = 14;
  std::vector<uint8_t> vectData;
  request->data_to_send = vectData;

  auto fp = std::bind(&EncodersComponent::read_cb, this, _1);
  call_i2c_command_service(request, fp); 

}

}  // namespace encoders

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(encoders::EncodersComponent)

