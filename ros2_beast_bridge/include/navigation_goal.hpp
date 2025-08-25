// navigation_goal.hpp
#ifndef NAVIGATION_GOAL_HPP
#define NAVIGATION_GOAL_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nlohmann/json.hpp"
#include "base_link_pose.hpp" 
#include <mutex>
#include <vector>
#include <string>
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp" 
#include <atomic>
#include <condition_variable>
#include "nav2_msgs/action/navigate_to_pose.hpp" 
#include "tf2/utils.h" 
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 
#include <memory> 
#include "std_msgs/msg/float32_multi_array.hpp"  // 引入Float32MultiArray类型
#include "rclcpp/subscription.hpp" 
#include <mutex> 
#include <boost/beast.hpp>

namespace beast = boost::beast;
namespace http = beast::http;
using json = nlohmann::json;
// 导航目标处理结果结构体
struct NavigationResult {
    bool successed;          // 处理是否成功
    int errCode;             // 错误码（0为成功）
    std::string msg;         // 结果消息
    std::string data;
    std::vector<std::pair<int, int>> grid_phits;  // 路径栅格坐标列表
};
// 超声波数据目标处理结果结构体
struct ObstacleResult {
    bool successed;          // 接口调用是否成功
    int errCode;             // 错误码（0为成功）
    std::string msg;         // 结果消息
    std::string data;        // 8位障碍状态（1有障碍/0无障碍）
    std::string raw_data;    // 原始超声波数据（逗号分隔字符串）
};
//机器人底盘数据结构体
struct RobotHardwareStatus {
    // 按要求定义所有字段
    int anticollision = 0;          // 防撞条状态：0未触发
    int battery_percentage = 66;    // 电量百分比
    int camera_num = 1;             // 相机数量
    int cameras = 0;                // 相机状态（未使用）
    int cameras_obs = 0;            // 相机障碍物（未使用）
    int charge = 0;                 // 充电状态：0未充电
    std::string current_map_name = ""; // 当前建图名称（空）
    int current_working = 3;        // 工作状态：3导航启动
    bool emergency_stop = false;    // 急停状态：未触发
    int type = 1;                   // 底盘类型：1差速底盘
    int ultrasonic = 1;             // 超声波状态
    int ultrasonic_num = 8;         // 超声波数量
};
// 导航目标处理器类
class NavigationGoalHandler {
public:
    // 构造函数：需传入节点指针和地图参数计算器
    explicit NavigationGoalHandler(
        rclcpp::Node::SharedPtr node,
        std::unique_ptr<BaseLinkPoseCalculator>& pose_calculator  // 改为接收 unique_ptr 引用
    );
    std::mutex& get_path_mutex() {
        return path_mutex_;
    }
    // 处理导航目标请求的核心函数
    NavigationResult handle_goal(const nlohmann::json& req_data);
    const nav_msgs::msg::Path& get_latest_path() const {
        return latest_path_;
    }
        // 坐标转换工具函数：栅格X→物理X（米）
    double grid_to_physical_x(int grid_x, const MapParameters& map_params);
    // 坐标转换工具函数：栅格Y→物理Y（米）
    double grid_to_physical_y(int grid_y, const MapParameters& map_params);
    // 坐标转换工具函数：物理X→栅格X
    int physical_to_grid_x(double x, const MapParameters& map_params);
    // 坐标转换工具函数：物理Y→栅格Y
    int physical_to_grid_y(double y, const MapParameters& map_params);
    NavigationResult plan_path(const nlohmann::json& req_data);
    NavigationResult handle_multi_goals(const nlohmann::json& req_data);
    NavigationResult stop_all_tasks();

    struct NavigationStatus {
        int status;          // 1:完成, 2:取消, 3:正在导航中
        size_t remaining;    // 剩余目标点数量
        std::string message; // 状态描述
    };
    NavigationStatus get_navigation_status();  

    ObstacleResult identify_obstacles();
    ObstacleResult pause_navigation();
    ObstacleResult resume_navigation();
    ObstacleResult set_init_pose(const nlohmann::json& req_data);
    void handle_robot_hardware_status(
        const http::request<http::string_body>& req,
        http::response<http::dynamic_body>& res
    );
    void handle_robot_local_status(
        const http::request<http::string_body>& req,
        http::response<http::dynamic_body>& res
    );
private:
    // ROS节点指针
    rclcpp::Node::SharedPtr node_;
    // 地图参数计算器（用于获取地图原点、分辨率等）
    std::unique_ptr<BaseLinkPoseCalculator>& pose_calculator_;

    // 导航目标发布者（发布到Nav2的/goal_pose话题）
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    // 全局路径订阅者（订阅Nav2规划的/plan话题）
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    // 路径数据互斥锁（线程安全）
    mutable std::mutex path_mutex_;
    nav_msgs::msg::Path latest_path_;          // 私有路径缓存
    // 全局路径回调函数（更新最新路径）
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr path_planner_client_;

    struct MultiGoalState {
        std::vector<nav2_msgs::action::NavigateToPose::Goal> goals;
        size_t current_index = 0;
        bool loop = false;
        int remaining_loops = 0;
        std::atomic<bool> is_running = false;
        std::atomic<bool> is_canceled = false;
        std::atomic<bool> is_paused = false;
        // 新增：当前目标句柄，用于精准取消
        std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> current_goal_handle;
    };
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_client_;
    std::unique_ptr<MultiGoalState> multi_goal_state_;  // 多目标状态
    std::timed_mutex multi_goal_mutex_;  // 多目标状态互斥锁
    std::condition_variable multi_goal_cv_;  // 导航完成条件变量
    //导航结果回调（用于触发下一个目标）
    void goal_response_callback(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle
    );
    //执行下一个目标点
    void execute_next_goal();
    void cancel_current_goal();

    std::vector<float> latest_ultrasound_data_;
    std::mutex ultrasound_mutex_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr ultrasound_sub_;
    void ultrasound_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    nav2_msgs::action::NavigateToPose::Goal paused_goal_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;  // 新增
};

#endif  // NAVIGATION_GOAL_HPP