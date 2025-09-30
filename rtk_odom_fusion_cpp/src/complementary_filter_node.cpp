#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp> // 关键：补充TwistWithCovariance头文件
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <cmath>
#include <memory>

class ComplementaryFilterNode : public rclcpp::Node {
public:
    ComplementaryFilterNode() : Node("complementary_filter"), tf_broadcaster_(this) {
        // 参数声明
        this->declare_parameter("alpha", 0.05);           
        this->declare_parameter("gps_min_accuracy", 0.1); 
        this->declare_parameter("gps_max_accuracy", 3.0); 
        this->declare_parameter("publish_tf", true);      
        this->declare_parameter("map_frame_id", "map");   
        this->declare_parameter("base_frame_id", "base_link");
        this->declare_parameter("use_2d", true);          
        this->declare_parameter("odom_topic", "/odom");   
        this->declare_parameter("gps_odom_topic", "/gps/odometry");

        // 参数读取
        alpha_ = this->get_parameter("alpha").as_double();
        gps_min_accuracy_ = this->get_parameter("gps_min_accuracy").as_double();
        gps_max_accuracy_ = this->get_parameter("gps_max_accuracy").as_double();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        map_frame_id_ = this->get_parameter("map_frame_id").as_string();
        base_frame_id_ = this->get_parameter("base_frame_id").as_string();
        use_2d_ = this->get_parameter("use_2d").as_bool();
        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string gps_odom_topic = this->get_parameter("gps_odom_topic").as_string();

        // 状态初始化
        fused_pose_.setIdentity();
        last_odom_time_ = this->now();
        gps_initialized_ = false;
        odom_initialized_ = false;
        // 初始化odom_last_twist_（避免未初始化赋值）
        odom_last_twist_ = geometry_msgs::msg::TwistWithCovariance();

        // 订阅器
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 100,
            std::bind(&ComplementaryFilterNode::odom_callback, this, std::placeholders::_1)
        );

        gps_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            gps_odom_topic, 10,
            std::bind(&ComplementaryFilterNode::gps_callback, this, std::placeholders::_1)
        );

        // 发布器
        fused_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom/fused", 100);
        status_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/fusion/status", 10);

        RCLCPP_INFO(this->get_logger(), "互补滤波节点启动完成！");
        RCLCPP_INFO(this->get_logger(), "全局坐标系: %s, 滤波系数α: %.2f", map_frame_id_.c_str(), alpha_);
    }

private:
    // 里程计高频回调（预测）
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg) {
        // 时间间隔计算
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_odom_time_).seconds();
        if (dt <= 0.0 || dt > 0.1) {
            last_odom_time_ = current_time;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "里程计时间间隔异常: %.3fs，跳过此次预测", dt);
            return;
        }

        // 里程计数据有效性检查（修复警告：使用linear_vel和angular_vel）
        double linear_vel = sqrt(pow(odom_msg->twist.twist.linear.x, 2) + 
                                 pow(odom_msg->twist.twist.linear.y, 2));
        double angular_vel = fabs(odom_msg->twist.twist.angular.z);
        // 新增：速度过大时拒绝（避免异常数据）
        if (linear_vel > 2.0 || angular_vel > 3.14) { // 假设最大线速2m/s，最大角速度π rad/s
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "里程计速度异常: 线速=%.2f, 角速度=%.2f", linear_vel, angular_vel);
            last_odom_time_ = current_time;
            return;
        }
        // 协方差检查
        if (odom_msg->pose.covariance[0] > 0.1 || odom_msg->pose.covariance[7] > 0.1) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                "里程计协方差过大，不可靠");
            last_odom_time_ = current_time;
            return;
        }

        // 首次初始化
        if (!odom_initialized_) {
            initialize_with_odom(odom_msg);
            // 初始化时更新速度（修复odom_last_twist_未赋值）
            odom_last_twist_ = odom_msg->twist;
            last_odom_time_ = current_time;
            publish_fused_data(current_time);
            return;
        }

        // 更新里程计速度（关键：每次收到里程计都更新，避免未赋值）
        odom_last_twist_ = odom_msg->twist;

        // 预测融合位姿
        predict_fused_pose(odom_msg, dt);

        // 发布高频融合结果
        publish_fused_data(current_time);

        // 更新时间戳
        last_odom_time_ = current_time;
    }

    // GPS低频回调（校正）
    void gps_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& gps_msg) {
        // GPS数据有效性检查
        if (!is_valid_gps(gps_msg)) {
            return;
        }

        // 提取GPS的map系位置
        Eigen::Vector3d gps_pos(
            gps_msg->pose.pose.position.x,
            gps_msg->pose.pose.position.y,
            use_2d_ ? 0.0 : gps_msg->pose.pose.position.z
        );

        // 首次GPS校正
        if (!gps_initialized_) {
            fused_pose_.translation() = gps_pos;
            gps_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "GPS首次校正完成！初始位置: (%.2f, %.2f)", 
                       gps_pos.x(), gps_pos.y());
            publish_fused_data(this->now());
            return;
        }

        // 自适应调整滤波系数α
        double gps_accuracy = calculate_gps_accuracy(gps_msg->pose.covariance);
        double adaptive_alpha = get_adaptive_alpha(gps_accuracy);

        // 互补滤波融合
        fused_pose_.translation() = (1.0 - adaptive_alpha) * fused_pose_.translation() + 
                                   adaptive_alpha * gps_pos;

        RCLCPP_DEBUG(this->get_logger(), "GPS校正完成！α=%.3f, 融合位置: (%.2f, %.2f)", 
                    adaptive_alpha, fused_pose_.translation().x(), fused_pose_.translation().y());
        publish_fused_data(this->now());
    }

    // GPS数据有效性检查
    bool is_valid_gps(const nav_msgs::msg::Odometry::ConstSharedPtr& gps_msg) {
        // 坐标系检查
        if (gps_msg->header.frame_id != map_frame_id_) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                "GPS坐标系错误！期望: %s, 实际: %s", 
                                map_frame_id_.c_str(), gps_msg->header.frame_id.c_str());
            return false;
        }

        // 精度检查
        double gps_accuracy = calculate_gps_accuracy(gps_msg->pose.covariance);
        if (gps_accuracy > gps_max_accuracy_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                "GPS精度不足: %.2fm（阈值: %.2fm）", 
                                gps_accuracy, gps_max_accuracy_);
            return false;
        }

        // 位置跳变检查
        if (gps_initialized_) {
            Eigen::Vector3d last_gps_pos = fused_pose_.translation();
            Eigen::Vector3d current_gps_pos(
                gps_msg->pose.pose.position.x,
                gps_msg->pose.pose.position.y,
                0.0
            );
            double pos_diff = (current_gps_pos - last_gps_pos).norm();
            if (pos_diff > 2.0) {
                RCLCPP_ERROR(this->get_logger(), "GPS位置跳变过大: %.2fm，拒绝校正", pos_diff);
                return false;
            }
        }

        return true;
    }

    // 计算GPS精度（HDOP）
    double calculate_gps_accuracy(const std::array<double, 36>& gps_cov) {
        double x_var = gps_cov[0] > 0 ? gps_cov[0] : 0.1;
        double y_var = gps_cov[7] > 0 ? gps_cov[7] : 0.1;
        return sqrt(x_var + y_var);
    }

    // 自适应调整滤波系数α
    double get_adaptive_alpha(double gps_accuracy) {
        double normalized_accuracy = std::clamp(
            (gps_accuracy - gps_min_accuracy_) / (gps_max_accuracy_ - gps_min_accuracy_),
            0.0, 1.0
        );
        double adaptive_alpha = alpha_ * (1.0 - normalized_accuracy);
        return std::clamp(adaptive_alpha, 0.02, 0.2);
    }

    // 用里程计初始化融合位姿
    void initialize_with_odom(const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg) {
        fused_pose_.translation() = Eigen::Vector3d(
            odom_msg->pose.pose.position.x,
            odom_msg->pose.pose.position.y,
            use_2d_ ? 0.0 : odom_msg->pose.pose.position.z
        );
        Eigen::Quaterniond odom_quat(
            odom_msg->pose.pose.orientation.w,
            odom_msg->pose.pose.orientation.x,
            odom_msg->pose.pose.orientation.y,
            odom_msg->pose.pose.orientation.z
        );
        fused_pose_.linear() = odom_quat.toRotationMatrix();

        odom_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "里程计初始化完成！初始位置: (%.2f, %.2f)", 
                   fused_pose_.translation().x(), fused_pose_.translation().y());
    }

    // 基于里程计速度预测融合位姿
    void predict_fused_pose(const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg, double dt) {
        // 提取里程计速度
        double vx = odom_msg->twist.twist.linear.x;
        double vy = odom_msg->twist.twist.linear.y;
        double wz = odom_msg->twist.twist.angular.z;

        // 转换为map系速度
        Eigen::Matrix3d rotation = fused_pose_.linear();
        Eigen::Vector3d body_vel(vx, vy, 0.0);
        Eigen::Vector3d map_vel = rotation * body_vel;

        // 预测位置
        fused_pose_.translation() += map_vel * dt;

        // 预测姿态（2D）
        if (use_2d_ && fabs(wz) > 1e-6) {
            Eigen::AngleAxisd yaw_rot(wz * dt, Eigen::Vector3d::UnitZ());
            fused_pose_.linear() = yaw_rot * rotation;
        }
    }

    // 发布融合位姿和TF
    void publish_fused_data(const rclcpp::Time& current_time) {
        // 发布融合里程计
        auto fused_odom = std::make_unique<nav_msgs::msg::Odometry>();
        fused_odom->header.stamp = current_time;
        fused_odom->header.frame_id = map_frame_id_;
        fused_odom->child_frame_id = base_frame_id_;
        // 位置
        fused_odom->pose.pose.position.x = fused_pose_.translation().x();
        fused_odom->pose.pose.position.y = fused_pose_.translation().y();
        fused_odom->pose.pose.position.z = fused_pose_.translation().z();
        // 姿态
        Eigen::Quaterniond fused_quat(fused_pose_.linear());
        fused_odom->pose.pose.orientation.x = fused_quat.x();
        fused_odom->pose.pose.orientation.y = fused_quat.y();
        fused_odom->pose.pose.orientation.z = fused_quat.z();
        fused_odom->pose.pose.orientation.w = fused_quat.w();
        // 速度（复用里程计速度，已在odom_callback中更新）
        fused_odom->twist = odom_last_twist_;
        // 协方差
        set_fused_covariance(fused_odom->pose.covariance);

        fused_odom_pub_->publish(std::move(fused_odom));

        // 发布TF
        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped map_to_base_tf;
            map_to_base_tf.header.stamp = current_time;
            map_to_base_tf.header.frame_id = map_frame_id_;
            map_to_base_tf.child_frame_id = base_frame_id_;
            // 平移
            map_to_base_tf.transform.translation.x = fused_pose_.translation().x();
            map_to_base_tf.transform.translation.y = fused_pose_.translation().y();
            map_to_base_tf.transform.translation.z = fused_pose_.translation().z();
            // 旋转
            map_to_base_tf.transform.rotation.x = fused_quat.x();
            map_to_base_tf.transform.rotation.y = fused_quat.y();
            map_to_base_tf.transform.rotation.z = fused_quat.z();
            map_to_base_tf.transform.rotation.w = fused_quat.w();

            tf_broadcaster_.sendTransform(map_to_base_tf);
        }

        // 发布状态
        publish_fusion_status(current_time);
    }

    // 设置融合协方差
    void set_fused_covariance(std::array<double, 36>& covariance) {
        covariance.fill(0.0);
        double gps_var = 0.1 * alpha_;
        double odom_var = 0.01 * (1 - alpha_);
        covariance[0] = gps_var + odom_var;  // x方差
        covariance[7] = gps_var + odom_var;  // y方差
        covariance[14] = use_2d_ ? 100.0 : 1.0; // z方差
        covariance[21] = 0.1; // rx方差
        covariance[28] = 0.1; // ry方差
        covariance[35] = 0.05; // rz方差
    }

    // 发布融合状态
    void publish_fusion_status(const rclcpp::Time& current_time) {
        auto status_msg = std::make_unique<nav_msgs::msg::Odometry>();
        status_msg->header.stamp = current_time;
        status_msg->header.frame_id = "fusion_status";
        // position字段：状态
        status_msg->pose.pose.position.x = alpha_;                    // 基础α
        status_msg->pose.pose.position.y = gps_initialized_ ? 1.0 : 0.0; // GPS初始化
        status_msg->pose.pose.position.z = odom_initialized_ ? 1.0 : 0.0; // 里程计初始化
        // orientation字段：位置/精度
        status_msg->pose.pose.orientation.x = fused_pose_.translation().x(); // 融合x
        status_msg->pose.pose.orientation.y = fused_pose_.translation().y(); // 融合y
        status_msg->pose.pose.orientation.z = calculate_gps_accuracy_last_;  // 最后GPS精度
        status_msg->pose.pose.orientation.w = (this->now() - last_odom_time_).seconds() * 1000; // 里程计间隔(ms)

        status_pub_->publish(std::move(status_msg));
    }

    // 成员变量
    // 参数变量
    double alpha_;
    double gps_min_accuracy_;
    double gps_max_accuracy_;
    bool publish_tf_;
    std::string map_frame_id_;
    std::string base_frame_id_;
    bool use_2d_;

    // 状态变量
    Eigen::Affine3d fused_pose_;
    rclcpp::Time last_odom_time_;
    geometry_msgs::msg::TwistWithCovariance odom_last_twist_; // 已初始化+赋值
    bool gps_initialized_;
    bool odom_initialized_;
    double calculate_gps_accuracy_last_ = 0.0;

    // ROS2组件
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr status_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ComplementaryFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}