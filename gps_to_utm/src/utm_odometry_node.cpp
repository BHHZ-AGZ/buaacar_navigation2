#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>  // 替换PoseStamped发布状态
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <mutex>
#include <array>

// 定义宏以确保M_PI可用
#define _GNU_SOURCE

class TwoDUTMTransformer : public rclcpp::Node {
public:
    TwoDUTMTransformer() : Node("utm_transformer"), tf_broadcaster_(this) {
        RCLCPP_INFO(this->get_logger(), "开始初始化二维UTM Transformer节点");
        
        try {
            // 参数声明
            this->declare_parameter("antenna_offset_x", 0.0);
            this->declare_parameter("antenna_offset_y", 0.0);
            this->declare_parameter("antenna_yaw", 0.0);  // 单位：度
            this->declare_parameter("auto_set_origin", true);
            this->declare_parameter("manual_origin_lat", 0.0);
            this->declare_parameter("manual_origin_lon", 0.0);
            this->declare_parameter("publish_tf", true);
            this->declare_parameter("utm_zone", 0);
            this->declare_parameter("northp", true);
            this->declare_parameter("covariance_scale", 1.0);
            this->declare_parameter("min_accuracy", 0.05);
            this->declare_parameter("max_accuracy", 5.0);
            
            RCLCPP_INFO(this->get_logger(), "参数声明完成");
            
            // 订阅GPS话题
            gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                "/gps/fix", 10, std::bind(&TwoDUTMTransformer::gps_callback, this, std::placeholders::_1));
            
            // 发布UTM坐标话题
            utm_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/gps/odometry", 10);
            
            // 发布PoseStamped用于可视化
            pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/gps/pose", 10);
            
            // 发布状态话题（改用Float64MultiArray避免无效坐标系）
            status_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/gps/status", 10);
            
            RCLCPP_INFO(this->get_logger(), "二维UTM Transformer节点初始化完成");
        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "初始化过程中发生异常: %s", e.what());
            throw;
        }
    }

private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "收到GPS数据: 纬度=%.6f, 经度=%.6f", 
                    msg->latitude, msg->longitude);
        
        std::lock_guard<std::mutex> lock(transform_mutex_);
        
        // 检查GPS数据是否有效
        if (msg->status.status < 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                "收到无效的GPS数据，状态: %d", msg->status.status);
            return;
        }
        
        // 检查经纬度是否在合理范围内
        if (std::abs(msg->latitude) > 90.0 || std::abs(msg->longitude) > 180.0) {
            RCLCPP_WARN(this->get_logger(), "收到无效的经纬度: 纬度=%.6f, 经度=%.6f", 
                       msg->latitude, msg->longitude);
            return;
        }
        
        try {
            // 获取参数值
            double antenna_offset_x = this->get_parameter("antenna_offset_x").as_double();
            double antenna_offset_y = this->get_parameter("antenna_offset_y").as_double();
            double antenna_yaw = this->get_parameter("antenna_yaw").as_double();  // 度
            bool auto_set_origin = this->get_parameter("auto_set_origin").as_bool();
            double manual_origin_lat = this->get_parameter("manual_origin_lat").as_double();
            double manual_origin_lon = this->get_parameter("manual_origin_lon").as_double();
            
            RCLCPP_DEBUG(this->get_logger(), "参数获取完成");
            
            // 设置原点（第一次接收数据或手动设置）
            if (!origin_set_) {
                if (auto_set_origin) {
                    // 自动设置原点为第一个有效点
                    set_origin(msg->latitude, msg->longitude);
                    RCLCPP_INFO(this->get_logger(), "自动设置原点: 纬度=%.6f, 经度=%.6f", 
                               msg->latitude, msg->longitude);
                } else if (manual_origin_lat != 0.0 && manual_origin_lon != 0.0) {
                    // 手动设置原点
                    set_origin(manual_origin_lat, manual_origin_lon);
                    RCLCPP_INFO(this->get_logger(), "手动设置原点: 纬度=%.6f, 经度=%.6f", 
                               manual_origin_lat, manual_origin_lon);
                } else {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                        "原点未设置，等待有效数据");
                    return;
                }
            }
            
            // 检查 local_cartesian_ 是否已初始化
            if (!local_cartesian_) {
                RCLCPP_ERROR(this->get_logger(), "LocalCartesian 未初始化");
                return;
            }
            
            // 使用GeographicLib进行高精度UTM转换
            double x, y, z;
            local_cartesian_->Forward(msg->latitude, msg->longitude, 0.0, x, y, z);
            
            RCLCPP_DEBUG(this->get_logger(), "UTM转换完成: x=%.2f, y=%.2f", x, y);
            
            // 应用天线偏移和旋转（仅二维）
            apply_antenna_offset_2d(x, y, antenna_offset_x, antenna_offset_y, antenna_yaw);
            
            // 发布转换后的数据
            publish_utm_data(msg, x, y);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "GPS数据处理异常: %s", e.what());
        }
    }
    
    void set_origin(double lat, double lon) {
        RCLCPP_INFO(this->get_logger(), "开始设置原点: 纬度=%.6f, 经度=%.6f", lat, lon);
        
        try {
            // 确定UTM区域
            int zone;
            bool northp;
            if (this->get_parameter("utm_zone").as_int() != 0) {
                zone = this->get_parameter("utm_zone").as_int();
                northp = this->get_parameter("northp").as_bool();
            } else {
                // 自动确定UTM区域
                zone = GeographicLib::UTMUPS::StandardZone(lat, lon);
                northp = lat >= 0;
            }
            
            RCLCPP_DEBUG(this->get_logger(), "UTM区域确定: 区域=%d, 北半球=%s", zone, northp ? "是" : "否");
            
            // 创建局部笛卡尔坐标系（高度始终为0）
            local_cartesian_ = std::make_unique<GeographicLib::LocalCartesian>(
                lat, lon, 0.0, GeographicLib::Geocentric::WGS84());
            
            origin_lat_ = lat;
            origin_lon_ = lon;
            utm_zone_ = zone;
            northp_ = northp;
            origin_set_ = true;
            
            RCLCPP_INFO(this->get_logger(), "设置UTM原点完成: 纬度=%.6f, 经度=%.6f, 区域=%d, 北半球=%s", 
                       lat, lon, zone, northp ? "是" : "否");
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "设置原点异常: %s", e.what());
            throw;
        }
    }
    
    void publish_utm_data(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg, double x, double y) {
        // 获取参数值
        double covariance_scale = this->get_parameter("covariance_scale").as_double();
        double min_accuracy = this->get_parameter("min_accuracy").as_double();
        double max_accuracy = this->get_parameter("max_accuracy").as_double();
        bool publish_tf = this->get_parameter("publish_tf").as_bool();
        
        // 计算精度指标
        double accuracy = calculate_accuracy(gps_msg->position_covariance);
        accuracy = std::max(min_accuracy, std::min(accuracy, max_accuracy));
        
        // 创建并发布UTM消息
        auto utm_msg = std::make_unique<nav_msgs::msg::Odometry>();
        utm_msg->header.stamp = gps_msg->header.stamp;
        utm_msg->header.frame_id = "odom";
        utm_msg->child_frame_id = "base_link";
        
        // 设置位置（z坐标强制为0）
        utm_msg->pose.pose.position.x = x;
        utm_msg->pose.pose.position.y = y;
        utm_msg->pose.pose.position.z = 0.0;
        utm_msg->pose.pose.orientation.w = 1.0;
        
        // 设置协方差
        set_covariance_2d(utm_msg->pose.covariance, gps_msg->position_covariance, 
                         covariance_scale, accuracy);
        
        // 保存header（关键：解决段错误）
        std_msgs::msg::Header utm_header = utm_msg->header;
        
        utm_publisher_->publish(std::move(utm_msg));
        
        // 发布PoseStamped用于可视化（使用保存的header）
        auto pose_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
        pose_msg->header = utm_header;
        pose_msg->pose.position.x = x;
        pose_msg->pose.position.y = y;
        pose_msg->pose.position.z = 0.0;
        pose_msg->pose.orientation.w = 1.0;
        pose_publisher_->publish(std::move(pose_msg));
        
        // 发布状态信息
        publish_status(gps_msg->status.status, accuracy);
        
        // 发布TF变换
        if (publish_tf) {
            geometry_msgs::msg::TransformStamped transform;
            transform.header = utm_header;  // 使用保存的header
            transform.header.frame_id = "odom";
            transform.child_frame_id = "base_link";
            transform.transform.translation.x = x;
            transform.transform.translation.y = y;
            transform.transform.translation.z = 0.0;
            transform.transform.rotation.w = 1.0;
            
            tf_broadcaster_.sendTransform(transform);
        }
    }
    
    // 应用天线偏移和旋转（修复角度单位问题）
    void apply_antenna_offset_2d(double& x, double& y, 
                                double offset_x, double offset_y, double yaw_deg) {
        // 角度转弧度（关键修复）
        double yaw_rad = yaw_deg * M_PI / 180.0;
        double cy = cos(yaw_rad), sy = sin(yaw_rad);
        double dx = cy * offset_x - sy * offset_y;
        double dy = sy * offset_x + cy * offset_y;
        x -= dx;
        y -= dy;
    }
    
    double calculate_accuracy(const std::array<double, 9>& covariance) {
        // 处理异常协方差（避免负数或0）
        if (covariance[0] <= 0 || covariance[4] <= 0) {
            RCLCPP_WARN(this->get_logger(), "GPS协方差异常，使用默认精度");
            return 1.0;
        }
        return sqrt(covariance[0] + covariance[4]);
    }
    
    // 修复协方差矩阵配置
    void set_covariance_2d(std::array<double, 36>& output_covariance,
                          const std::array<double, 9>& input_covariance,
                          double scale, double accuracy) {
        output_covariance.fill(0.0);
        // x和y方向方差（仅设置对角线，符合ROS规范）
        output_covariance[0] = std::max(input_covariance[0] * scale, 0.01);  // x方差
        output_covariance[7] = std::max(input_covariance[4] * scale, 0.01);  // y方差
        // 其他维度设为较大值（表示不确定性高）
        output_covariance[14] = 100.0;  // z方差
        output_covariance[21] = 100.0;  // rx方差
        output_covariance[28] = 100.0;  // ry方差
        output_covariance[35] = 100.0;  // rz方差
        // 移除错误的output_covariance[2] = accuracy;
    }
    
    // 修复状态发布（使用Float64MultiArray，避免无效坐标系）
    void publish_status(int fix_status, double accuracy) {
        auto status_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
        // 数据含义：[定位状态, 精度, 原点是否设置, 原点纬度, 原点经度, UTM分区, 是否北半球]
        status_msg->data = {
            static_cast<double>(fix_status),
            accuracy,
            origin_set_ ? 1.0 : 0.0,
            origin_lat_,
            origin_lon_,
            static_cast<double>(utm_zone_),
            northp_ ? 1.0 : 0.0
        };
        status_publisher_->publish(std::move(status_msg));
    }
    
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr utm_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr status_publisher_;  // 类型修改
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    std::unique_ptr<GeographicLib::LocalCartesian> local_cartesian_;
    std::mutex transform_mutex_;
    
    bool origin_set_ = false;
    double origin_lat_ = 0.0;
    double origin_lon_ = 0.0;
    int utm_zone_ = 0;
    bool northp_ = true;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwoDUTMTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
    