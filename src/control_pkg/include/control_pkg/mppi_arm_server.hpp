#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

class MppiArmServer
: rclcpp::Node
{
public:
    MppiArmServer();

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState> joint_state_subscriber_;
};
