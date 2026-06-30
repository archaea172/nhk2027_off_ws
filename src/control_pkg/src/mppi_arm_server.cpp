#include "mppi_arm_server.hpp"

MppiArmServer::MppiArmServer()
: rclcpp::Node("mppi_arm_server")
{

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<MppiArmServer> node = std::make_shared<MppiArmServer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
