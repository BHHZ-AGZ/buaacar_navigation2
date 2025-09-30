#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "robot_navigation/srv/set_pose.hpp"

#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

// PID控制器结构体
struct PIDController {
    double kp = 0.0;   // 比例系数
    double ki = 0.0;   // 积分系数
    double kd = 0.0;   // 微分系数
    double integral = 0.0;  // 积分项
    double prev_error = 0.0; // 上一次误差
    double integral_limit = 1.0;  // 积分限幅

    // 计算PID输出
    double compute(double error, double dt) {
        // 积分项计算与限幅
        integral += error * dt;
        integral = std::clamp(integral, -integral_limit, integral_limit);

        // 微分项计算
        double derivative = (dt > 0) ? (error - prev_error) / dt : 0.0;

        // 保存当前误差作为下一次的前向误差
        prev_error = error;

        // PID输出
        return kp * error + ki * integral + kd * derivative;
    }

    // 重置PID状态
    void reset() {
        integral = 0.0;
        prev_error = 0.0;
    }
};

class DiffDrivePositionController : public rclcpp::Node {
public:
    DiffDrivePositionController() : Node("diff_drive_position_controller") {
        // 声明并获取参数
        declare_parameters();
        get_parameters();

        // 初始化PID控制器
        init_pid_controllers();

        // 订阅里程计信息（获取当前位置和速度）
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/fused", 10, std::bind(&DiffDrivePositionController::odom_callback, this, std::placeholders::_1));

        // 订阅目标位置（来自路径规划或手动设置）
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&DiffDrivePositionController::goal_callback, this, std::placeholders::_1));

        // 创建设置目标点的服务
        set_goal_srv_ = this->create_service<robot_navigation::srv::SetPose>(
            "/set_position_goal",
            std::bind(&DiffDrivePositionController::set_goal_service_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // 发布速度指令（控制底盘）
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // 控制循环定时器（10Hz，与位置环频率一致）
        control_timer_ = this->create_wall_timer(
            100ms, std::bind(&DiffDrivePositionController::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Diff Drive Position Controller initialized");
        RCLCPP_INFO(this->get_logger(), "Position tolerance: %.2fm, angle tolerance: %.2frad",
                   pos_tolerance_, angle_tolerance_);
    }

private:
    // 声明所有参数
    void declare_parameters() {
        // 位置环PID参数
        this->declare_parameter("position.kp_rho", 0.8);
        this->declare_parameter("position.kp_alpha", 2.0);
        this->declare_parameter("position.kp_beta", -0.5);
        
        // 速度环PID参数
        this->declare_parameter("velocity.kp_linear", 0.5);
        this->declare_parameter("velocity.ki_linear", 0.1);
        this->declare_parameter("velocity.kd_linear", 0.05);
        this->declare_parameter("velocity.kp_angular", 0.8);
        this->declare_parameter("velocity.ki_angular", 0.2);
        this->declare_parameter("velocity.kd_angular", 0.1);
        
        // 底盘参数
        this->declare_parameter("base.wheel_base", 0.3);  // 轮距(m)
        this->declare_parameter("base.max_linear_vel", 0.5);  // 最大线速度(m/s)
        this->declare_parameter("base.max_angular_vel", 1.5); // 最大角速度(rad/s)
        
        // 控制精度参数
        this->declare_parameter("control.pos_tolerance", 0.05);  // 位置容忍误差(m)
        this->declare_parameter("control.angle_tolerance", 0.05); // 角度容忍误差(rad)
    }

    // 获取参数值
    void get_parameters() {
        // 位置环参数
        this->get_parameter("position.kp_rho", kp_rho_);
        this->get_parameter("position.kp_alpha", kp_alpha_);
        this->get_parameter("position.kp_beta", kp_beta_);
        
        // 速度环参数
        this->get_parameter("velocity.kp_linear", linear_pid_.kp);
        this->get_parameter("velocity.ki_linear", linear_pid_.ki);
        this->get_parameter("velocity.kd_linear", linear_pid_.kd);
        this->get_parameter("velocity.kp_angular", angular_pid_.kp);
        this->get_parameter("velocity.ki_angular", angular_pid_.ki);
        this->get_parameter("velocity.kd_angular", angular_pid_.kd);
        
        // 底盘参数
        this->get_parameter("base.wheel_base", wheel_base_);
        this->get_parameter("base.max_linear_vel", max_linear_vel_);
        this->get_parameter("base.max_angular_vel", max_angular_vel_);
        
        // 控制精度
        this->get_parameter("control.pos_tolerance", pos_tolerance_);
        this->get_parameter("control.angle_tolerance", angle_tolerance_);
    }

    // 初始化PID控制器
    void init_pid_controllers() {
        linear_pid_.integral_limit = 0.5;  // 线速度积分限幅
        angular_pid_.integral_limit = 1.0; // 角速度积分限幅
    }

    // 里程计回调：更新当前位置和速度
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = msg->pose.pose;
        current_linear_vel_ = msg->twist.twist.linear.x;
        current_angular_vel_ = msg->twist.twist.angular.z;
        last_odom_time_ = this->get_clock()->now();
    }

    // 目标位置回调：更新目标位置
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        goal_pose_ = msg->pose;
        goal_reached_ = false;
        linear_pid_.reset();  // 重置PID状态
        angular_pid_.reset();
        RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f)",
                   goal_pose_.position.x, goal_pose_.position.y);
    }

    // 设置目标点服务回调
    void set_goal_service_callback(
        const std::shared_ptr<robot_navigation::srv::SetPose::Request> request,
        std::shared_ptr<robot_navigation::srv::SetPose::Response> response) {
        goal_pose_ = request->pose;
        goal_reached_ = false;
        linear_pid_.reset();
        angular_pid_.reset();
        
        response->success = true;
        response->message = "Position goal set successfully";
        RCLCPP_INFO(this->get_logger(), "Goal set via service: (%.2f, %.2f)",
                   goal_pose_.position.x, goal_pose_.position.y);
    }

    // 角度归一化到[-π, π]
    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

    // 从四元数获取偏航角
    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& quat) {
        tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    // 控制主循环
    void control_loop() {
        // 检查是否有目标位置和里程计数据
        if (last_odom_time_.nanoseconds() == 0 || goal_reached_) {
            return;
        }

        // 计算控制周期
        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - last_odom_time_).seconds();
        last_odom_time_ = current_time;

        // 1. 位置环计算：从位置误差得到期望速度
        double current_x = current_pose_.position.x;
        double current_y = current_pose_.position.y;
        double current_yaw = get_yaw_from_quaternion(current_pose_.orientation);

        double target_x = goal_pose_.position.x;
        double target_y = goal_pose_.position.y;
        double target_yaw = get_yaw_from_quaternion(goal_pose_.orientation);

        // 计算位置误差
        double dx = target_x - current_x;
        double dy = target_y - current_y;
        double rho = std::hypot(dx, dy);  // 距离误差

        double alpha = normalize_angle(std::atan2(dy, dx) - current_yaw);  // 角度误差
        double beta = normalize_angle(-current_yaw - alpha + target_yaw);   // 朝向误差

        // 检查是否到达目标
        if (rho < pos_tolerance_ && std::abs(alpha) < angle_tolerance_ && 
            std::abs(beta) < angle_tolerance_) {
            stop_robot();
            goal_reached_ = true;
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            return;
        }

        // 位置环输出：计算期望速度（P控制）
        double desired_linear_vel = kp_rho_ * rho;
        double desired_angular_vel = kp_alpha_ * alpha + kp_beta_ * beta;

        // 速度限幅
        desired_linear_vel = std::clamp(desired_linear_vel, -max_linear_vel_, max_linear_vel_);
        desired_angular_vel = std::clamp(desired_angular_vel, -max_angular_vel_, max_angular_vel_);

        // 2. 速度环计算：从速度误差得到控制输出
        double linear_error = desired_linear_vel - current_linear_vel_;
        double angular_error = desired_angular_vel - current_angular_vel_;

        double linear_output = linear_pid_.compute(linear_error, dt);
        double angular_output = angular_pid_.compute(angular_error, dt);

        // 3. 发布速度指令
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = linear_output;
        cmd_vel.angular.z = angular_output;
        cmd_vel_pub_->publish(cmd_vel);

        // 调试信息
        // RCLCPP_DEBUG(this->get_logger(), "rho: %.2f, alpha: %.2f, beta: %.2f", rho, alpha, beta);
        // RCLCPP_DEBUG(this->get_logger(), "desired v: %.2f, desired w: %.2f", desired_linear_vel, desired_angular_vel);
    }

    // 停止机器人
    void stop_robot() {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
    }

    // 成员变量
    // 订阅者和发布者
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Service<robot_navigation::srv::SetPose>::SharedPtr set_goal_srv_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // 状态变量
    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Pose goal_pose_;
    double current_linear_vel_ = 0.0;
    double current_angular_vel_ = 0.0;
    rclcpp::Time last_odom_time_;
    bool goal_reached_ = false;

    // PID控制器
    PIDController linear_pid_;    // 线速度PID
    PIDController angular_pid_;   // 角速度PID

    // 控制参数
    double kp_rho_;       // 位置环-距离比例系数
    double kp_alpha_;     // 位置环-角度比例系数
    double kp_beta_;      // 位置环-朝向比例系数
    double wheel_base_;   // 轮距(m)
    double max_linear_vel_;   // 最大线速度(m/s)
    double max_angular_vel_;  // 最大角速度(rad/s)
    double pos_tolerance_;    // 位置容忍误差(m)
    double angle_tolerance_;  // 角度容忍误差(rad)
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffDrivePositionController>());
    rclcpp::shutdown();
    return 0;
}
