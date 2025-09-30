// base_link_pose.hpp
#ifndef BASE_LINK_POSE_HPP
#define BASE_LINK_POSE_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2/LinearMath/Transform.h>
#include <optional>
#include <mutex>  // 新增：引入互斥锁头文件

// 栅格姿态结构体
struct GridPose {
    int grid_x;
    int grid_y;
    double yaw_deg;  // 朝向角（度，0~360）
    double x;        // 物理x坐标（米）
    double y;        // 物理y坐标（米）
    bool in_map;     // 是否在地图范围内
};
struct MapParameters {
    double resolution;  // 分辨率（米/像素）
    int width;          // 宽度（像素）
    int height;         // 高度（像素）
    double origin_x;    // 原点X坐标（米）
    double origin_y;    // 原点Y坐标（米）
};
class BaseLinkPoseCalculator {
public:
    explicit BaseLinkPoseCalculator(rclcpp::Node::SharedPtr node);
    std::optional<GridPose> get_current_grid_pose();
     MapParameters get_map_parameters() const; 
    bool is_map_loaded() const {
        std::lock_guard<std::mutex> lock(map_mutex_);
        return map_received_;  // map_received_是地图回调中设置的标志位
    }
private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    // 节点指针
    rclcpp::Node::SharedPtr node_;

    // TF相关
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 地图相关
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    double map_resolution_ = 0.0;
    double map_origin_x_ = 0.0;
    double map_origin_y_ = 0.0;
    int map_width_ = 0;
    int map_height_ = 0;
    bool map_received_ = false;  // 地图是否已接收的标志位（已在地图回调中更新）
    mutable std::mutex map_mutex_; //保护地图的互斥锁
};

#endif  // BASE_LINK_POSE_HPP