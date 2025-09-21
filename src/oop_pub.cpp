#include "ws_tutorial/include/ws_tutorial/oop_pub.hpp"


using namespace std::chrono_literals;

OOP_PUB::OOP_PUB(): Node("oop_pub"),count_(0)
{
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic",10);
timer_ = this->create_wall_timer(
  500ms, std::bind(&OOP_PUB::timer_callback, this));
}

void OOP_PUB::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

int main(int argc,char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<OOP_PUB>());
    rclcpp::shutdown();
    return 0;
}