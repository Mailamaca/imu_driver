#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <atomic>


//#include "motors_interface/pca9685_motors.h"

#include "maila_msgs/msg/vehicle_control.hpp"
#include "maila_msgs/msg/vehicle_mode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;


class EncodersInterface : public rclcpp::Node
{
  public:

    EncodersInterface() : Node("encoders_interface")
    {
      declare_parameters();
      load_parameters_and_initialization();    

      create_publishers();
      create_subscriptions();
      create_timers();      
      
    }
  
  private:

    int file_i2c;

    // Topic name list
    std::string spur_gear_encoder_topic_name; // = "/hw/encoders/sg";

    int out_msg_period;
    
    //rclcpp::Subscription<maila_msgs::msg::VehicleMode>::SharedPtr subs_mode;

    rclcpp::TimerBase::SharedPtr timer_encoders_reading;

    rclcpp::Publisher<maila_msgs::msg::VehicleControl>::SharedPtr pub_spur_gear_encoder;
        
    void timer_encoders_reading_callback()
    {      
      int length;
      uint8_t buffer[2] = {0};
      uint8_t addr = 0x09;
    
      if (ioctl(file_i2c, I2C_SLAVE, addr) < 0){
        RCLCPP_WARN(this->get_logger(), "Failed to acquire bus access and/or talk to slave");
      }
      
      
      
      // ------------- WRITE BYTES -------------
      buffer[0] = 4;
      buffer[1] = 0;
      length = 1; //<<<<< Number of bytes to write
      if (write(file_i2c, buffer, length) != length){ // write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
        // ERROR HANDLING: i2c transaction failed
        RCLCPP_WARN(this->get_logger(), "Failed to write to the i2c bus");
      } else {
        // ------------ READ BYTES -------
        length = 2;
        if (read(file_i2c, buffer, length) != length){ // read() returns the number of bytes actually read, if it doesn't match then an error occurred (e.g. no response from the device)
          //ERROR HANDLING: i2c transaction failed
          RCLCPP_WARN(this->get_logger(), "Failed to read from the i2c bus");
        } else {
          RCLCPP_INFO(this->get_logger(), "Data read->\t:%d", buffer[0]);
        }
      } 


    }

    void create_timers() {

      start_timer_encoders_reading();
    }

    void start_timer_encoders_reading() {
      std::chrono::duration<int, std::milli> timeout_time(out_msg_period);
      timer_encoders_reading = this->create_wall_timer(
        timeout_time, std::bind(&EncodersInterface::timer_encoders_reading_callback, this)
        );
    }

    void create_subscriptions() {
      //subs_mode = this->create_subscription<maila_msgs::msg::VehicleMode>(
      //  mode_topic_name, 10, std::bind(&MotorsInterface::subs_mode_callback, this, _1));
    }

    void create_publishers() {
      pub_spur_gear_encoder = this->create_publisher<maila_msgs::msg::VehicleControl>(spur_gear_encoder_topic_name, 10);
      
    }

    void load_parameters_and_initialization() {

      this->get_parameter("topics.out_spur_gear_encoder", spur_gear_encoder_topic_name);
      this->get_parameter("out_msg_period", out_msg_period);


      char *filename = (char*)"/dev/i2c-1";
      if((file_i2c = open(filename, O_RDWR))< 0 ){
        //ERROR HANDLING: you can check errno to see what went wrong;
        RCLCPP_WARN(this->get_logger(), "Failed to open the i2c bus");
      }

    }

    void declare_parameters() {
      this->declare_parameter<std::string>("topics.out_spur_gear_encoder", "");
      this->declare_parameter<int>("out_msg_period", 1000);
    }
 
    
    

  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EncodersInterface>());
    rclcpp::shutdown();
    return 0;
  }
