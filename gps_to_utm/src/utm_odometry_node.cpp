/*
 * Translates sensor_msgs/NavSatFix into nav_msgs/Odometry using UTM (ROS 2适配版)
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>  // 新增：用于NavSatStatus常量
#include <gps_to_utm/conversions.h>            // 本地UTM转换头文件
#include <nav_msgs/msg/odometry.hpp>
#include <array>  // 替换boost::array为std::array

using namespace gps_common;

class UtmOdometryNode : public rclcpp::Node {
public:
    UtmOdometryNode() : Node("utm_odometry_node") {
        // 声明并获取参数
        this->declare_parameter<std::string>("frame_id", "map");
        this->declare_parameter<std::string>("child_frame_id", "base_link");
        this->declare_parameter<double>("rot_covariance", 99999.0);
        this->declare_parameter<bool>("append_zone", false);

        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("child_frame_id", child_frame_id_);
        this->get_parameter("rot_covariance", rot_cov_);
        this->get_parameter("append_zone", append_zone_);

        // 创建发布者和订阅者
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/gps", 10);
        fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", 10, std::bind(&UtmOdometryNode::fix_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "UTM Odometry Node started!");
    }

private:
    void fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr fix) {
        // 修复1：STATUS_NO_FIX属于NavSatStatus
        if (fix->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 60000, "No GPS fix available.");
            return;
        }

        if (fix->header.stamp.sec == 0 && fix->header.stamp.nanosec == 0) {
            RCLCPP_DEBUG(this->get_logger(), "Invalid GPS timestamp, skipping.");
            return;
        }

        // WGS84转UTM
        double northing, easting;
        std::string zone;
        LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

        // 构建Odometry消息
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = fix->header.stamp;

        // 设置坐标系
        if (frame_id_.empty()) {
            odom_msg.header.frame_id = append_zone_ ? fix->header.frame_id + "/utm_" + zone : fix->header.frame_id;
        } else {
            odom_msg.header.frame_id = append_zone_ ? frame_id_ + "/utm_" + zone : frame_id_;
        }
        odom_msg.child_frame_id = child_frame_id_;

        // 位置信息
        odom_msg.pose.pose.position.x = easting;
        odom_msg.pose.pose.position.y = northing;
        odom_msg.pose.pose.position.z = fix->altitude;

        // 姿态（默认值）
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = 0.0;
        odom_msg.pose.pose.orientation.w = 1.0;

        // 修复2：使用std::array而非boost::array
        std::array<double, 36> covariance = {{
            fix->position_covariance[0], fix->position_covariance[1], fix->position_covariance[2], 0, 0, 0,
            fix->position_covariance[3], fix->position_covariance[4], fix->position_covariance[5], 0, 0, 0,
            fix->position_covariance[6], fix->position_covariance[7], fix->position_covariance[8], 0, 0, 0,
            0, 0, 0, rot_cov_, 0, 0,
            0, 0, 0, 0, rot_cov_, 0,
            0, 0, 0, 0, 0, rot_cov_
        }};
        odom_msg.pose.covariance = covariance;  // 现在类型匹配

        odom_pub_->publish(odom_msg);
    }

    // 成员变量
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
    std::string frame_id_;
    std::string child_frame_id_;
    double rot_cov_;
    bool append_zone_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UtmOdometryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}