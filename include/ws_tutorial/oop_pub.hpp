#pragma once
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class OOP_PUB: public rclcpp::Node
{
    public:
    OOP_PUB();

    private:
        void callback_timer();
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;
        

};