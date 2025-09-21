#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc,char* argv[])
{
    rclcpp::init(argc,argv);

    std::shared_ptr<rclcpp::Node> pub_node=rclcpp::Node::make_shared("simple_node");
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> publisher = pub_node->create_publisher<std_msgs::msg::String>("string_pub",10);

    rclcpp::WallRate loop_rate(2);

    while(rclcpp::ok())
    {
        std_msgs::msg::String msg;
        msg.data = "Hollow world";
        RCLCPP_INFO(pub_node->get_logger(),"Publishing :  '%s' ",msg.data.c_str());
        publisher ->publish(msg);
        rclcpp::spin_some(pub_node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}