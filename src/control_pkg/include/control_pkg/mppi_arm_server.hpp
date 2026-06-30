#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

#include "mppi_arm.hpp"
#include <Eigen/Dense>

class MppiArmServer
: public rclcpp::Node
{
public:
    MppiArmServer();

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
};
