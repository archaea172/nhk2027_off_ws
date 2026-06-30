#include "mppi_arm_server.hpp"

MppiArmServer::MppiArmServer()
: rclcpp::Node("mppi_arm_server")
{

}

void MppiArmServer::robot_description_callback(const std_msgs::msg::String::SharedPtr rxdata)
{
    this->is_subscription_robot_description_ = true;

    this->urdf_ = rxdata->data;
}

void MppiArmServer::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr rxdata)
{
    this->joint_data_ = *rxdata;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<MppiArmServer> node = std::make_shared<MppiArmServer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
