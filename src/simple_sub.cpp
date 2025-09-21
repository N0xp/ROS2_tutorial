#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"



void topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "I heard: '%s'", msg->data.c_str());}

int main(int argc , char* argv[])
{

    rclcpp::init(argc,argv);
    std::shared_ptr<rclcpp::Node> sub_node = rclcpp::Node::make_shared("simple_subscriber");   
    auto subscription = sub_node->create_subscription<std_msgs::msg::String>(
    "string_pub", 10, topic_callback);
    rclcpp::spin(sub_node);
    rclcpp::shutdown();
    return 0;
}