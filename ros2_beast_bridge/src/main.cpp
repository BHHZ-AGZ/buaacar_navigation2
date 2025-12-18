#include "ros2_beast_bridge/http_server.hpp"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<HttpServer>(options);
    node->post_init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}