#include "base_link_pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  
#include <cmath>

// BaseLinkPoseCalculator 构造函数中初始化订阅
BaseLinkPoseCalculator::BaseLinkPoseCalculator(rclcpp::Node::SharedPtr node) : node_(node) {
    // 订阅地图（核心依赖）
    map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
        std::bind(&BaseLinkPoseCalculator::map_callback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(node_->get_logger(), "BaseLinkPoseCalculator 已初始化，开始订阅 /map");

    // 初始化 TF 缓冲（另一核心依赖）
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void BaseLinkPoseCalculator::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map_resolution_ = msg->info.resolution;
    map_origin_x_ = msg->info.origin.position.x;
    map_origin_y_ = msg->info.origin.position.y;
    map_width_ = msg->info.width;
    map_height_ = msg->info.height;
    map_received_ = true;
    
    RCLCPP_INFO(node_->get_logger(), "地图数据已加载: 分辨率=%.2fm, 尺寸=%dx%d栅格",
                map_resolution_, map_width_, map_height_);
}

std::optional<GridPose> BaseLinkPoseCalculator::get_current_grid_pose() {
    if (!map_received_) {
        // 增加等待提示，避免频繁警告
        static rclcpp::Time last_warn_time = node_->get_clock()->now();
        rclcpp::Duration warn_interval = rclcpp::Duration::from_seconds(5);
        if (node_->get_clock()->now() - last_warn_time > warn_interval) {
            RCLCPP_WARN(node_->get_logger(), "等待地图数据...（未收到/map话题）");
            last_warn_time = node_->get_clock()->now();
        }
        return std::nullopt;
    }
    
    try {
        // 获取map→odom变换
        auto map_to_odom = tf_buffer_->lookupTransform(
            "map", "odom",
            tf2::TimePointZero,
            std::chrono::milliseconds(100)
        );
        
        // 获取odom→base_link变换
        auto odom_to_base = tf_buffer_->lookupTransform(
            "odom", "base_link",
            tf2::TimePointZero,
            std::chrono::milliseconds(100)
        );
        
        // 转换为tf2::Transform类型进行组合
        tf2::Transform tf_map_odom;
        tf2::fromMsg(map_to_odom.transform, tf_map_odom);
        
        tf2::Transform tf_odom_base;
        tf2::fromMsg(odom_to_base.transform, tf_odom_base);
        
        // 组合变换：map→base_link = map→odom * odom→base_link
        tf2::Transform tf_map_base = tf_map_odom * tf_odom_base;
        
        // 转换为geometry_msgs::msg::Transform（现在有了转换模板支持）
        geometry_msgs::msg::Transform map_to_base_transform = tf2::toMsg(tf_map_base);
        
        // 提取位置信息
        double x = map_to_base_transform.translation.x;
        double y = map_to_base_transform.translation.y;
        
        // 计算栅格坐标
        int grid_x = static_cast<int>(std::round((x - map_origin_x_) / map_resolution_));
        int grid_y = static_cast<int>(std::round((y - map_origin_y_) / map_resolution_));
        grid_y = map_height_ - 1 - grid_y;
        
        // 检查是否在地图范围内
        bool in_map = (grid_x >= 0 && grid_x < map_width_) && 
                      (grid_y >= 0 && grid_y < map_height_);
        
        // 计算朝向角度
        tf2::Quaternion q(
            map_to_base_transform.rotation.x,
            map_to_base_transform.rotation.y,
            map_to_base_transform.rotation.z,
            map_to_base_transform.rotation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        // 转换为0~360度
        double yaw_deg = yaw * 180.0 / M_PI;
        if (yaw_deg < 0) {
            yaw_deg += 360.0;
        }
        
        // 构造返回结果
        GridPose result{
            .grid_x = grid_x,
            .grid_y = grid_y,
            .yaw_deg = yaw_deg,
            .x = x,
            .y = y,
            .in_map = in_map
        };
        
        return result;
        
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(node_->get_logger(), "TF变换获取失败: %s", ex.what());
        return std::nullopt;
    }
}
MapParameters BaseLinkPoseCalculator::get_map_parameters() const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return {
        map_resolution_,
        map_width_,
        map_height_,
        map_origin_x_,
        map_origin_y_
    };
}