#pragma once

#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <memory>
#include <thread>
#include <mutex>
#include <unordered_map>
#include <string>
#include <nlohmann/json.hpp>
#include "base_link_pose.hpp" 
#include "navigation_goal.hpp"

namespace beast = boost::beast;
namespace http = beast::http;
namespace net = boost::asio;
using json = nlohmann::json;
using tcp = boost::asio::ip::tcp;

class HttpServer : public rclcpp::Node {
public:
    explicit HttpServer(const rclcpp::NodeOptions& options);
    virtual ~HttpServer();
    std::string generateToken(const std::string& username);
    bool validateToken(const std::string& token);
    void post_init();
private:
    void startServer();
    void handleSession(tcp::socket socket);
    void processRequest(const http::request<http::string_body>& req, 
                       http::response<http::dynamic_body>& res);
    
    // 订阅不同消息类型的回调函数
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void stringCallback(const std_msgs::msg::String::SharedPtr msg);
    
    // 发布者
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;
    
    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_subscriber_;
    std::unique_ptr<BaseLinkPoseCalculator> pose_calculator_;

    std::thread server_thread_;
    net::io_context ioc_;
    std::thread ioc_thread_;
    unsigned short port_;
    std::unordered_map<std::string, std::string> valid_users_;  // 用户列表（私有）
    std::string jwt_secret_;          // JWT密钥（从配置文件读取）
    int token_expire_hours_;          // Token有效期（小时）
    std::mutex data_mutex_;         // 保护点云、字符串数据
    std::mutex imu_mutex_;          // 新增：保护IMU数据
    json imu_data_;
    json pointcloud_metadata_;
    std::string string_data_;
    std::shared_ptr<NavigationGoalHandler> nav_handler_;
    
};