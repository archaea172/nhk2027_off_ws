#include "mppi_arm.hpp"
#include <chrono>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc, char *argv[])
{
    MppiArmParams parameters;
    parameters.goal_pos << 0.5, 0.4, 1.57;
    parameters.control_frequency = 50;
    parameters.predict_resolution = 0.02;
    parameters.predict_horizon = 100;
    parameters.sample_num = 200;
    Eigen::Matrix3d cov;
    cov << 
    1, 0, 0,
    0, 1, 0,
    0, 0, 1;
    parameters.cov = cov;
    parameters.lambda = 1.0;
    parameters.gamma = 0;

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("test_mppi");
    std_msgs::msg::String robot_description;
    rclcpp::wait_for_message<std_msgs::msg::String>(
        robot_description,
        node,
        "robot_description",
        std::chrono::milliseconds(-1),
        rclcpp::QoS(1).transient_local().reliable());
    parameters.arm_parameters.urdf = robot_description.data;
    parameters.arm_parameters.base_link = "arm_base_v2_1";
    parameters.arm_parameters.end_link = "tcp_link";

    MppiArmController test_controller(parameters);
    rclcpp::shutdown();

    Eigen::Vector3d pos;
    pos << 1, 0.1, 1;

    std::chrono::system_clock::time_point  start, end; // 型は auto で可
    start = std::chrono::system_clock::now(); // 計測開始時間

    Eigen::Vector3d arm_pos = test_controller.controlLoop(pos);

    end = std::chrono::system_clock::now();  // 計測終了時間
    
    double elapsed = std::chrono::duration<double, std::milli>(end - start).count();
    printf("time:%f ms\r\n", elapsed);
    std::cout << arm_pos << std::endl;
}
