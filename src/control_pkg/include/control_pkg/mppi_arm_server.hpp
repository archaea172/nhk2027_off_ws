#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include "mppi_arm.hpp"
#include <Eigen/Dense>

class MppiArmServer
: public rclcpp::Node
{
public:
    MppiArmServer();

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr rxdata);
    void robot_description_callback(const std_msgs::msg::String::SharedPtr rxdata);

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscriber_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr j1_motor_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr j2_motor_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr j3_motor_publisher_;

    bool is_subscription_robot_description_{false};
    std::string urdf_;
    sensor_msgs::msg::JointState joint_data_;
};
