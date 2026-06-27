#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"

#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class IdeArmSimBridge
: public rclcpp::Node
{
public:
    IdeArmSimBridge();
private:
    void joint_array_control_callback(const std_msgs::msg::Float32MultiArray::SharedPtr rxdata, int index);

    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> joint_control_publisher_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr> control_subscriber_;
};