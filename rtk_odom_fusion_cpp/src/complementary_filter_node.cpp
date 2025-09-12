#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class ComplementaryFilterNode : public rclcpp::Node
{
public:
  ComplementaryFilterNode()
  : Node("complementary_filter_fusion"), 
    alpha_position_(0.05),
    alpha_orientation_(0.02),
    use_imu_for_orientation_(true)
  {
    // 声明参数
    this->declare_parameter<double>("alpha_position", 0.05);  //较小的值（如 0.01-0.1）使融合结果更信任里程计的高频变化
    this->declare_parameter<double>("alpha_orientation", 0.02); //较小的值（如 0.01-0.05）使融合结果更信任 IMU 的高频变化
    this->declare_parameter<bool>("use_imu_for_orientation", true);
    
    alpha_position_ = this->get_parameter("alpha_position").as_double();
    alpha_orientation_ = this->get_parameter("alpha_orientation").as_double();
    use_imu_for_orientation_ = this->get_parameter("use_imu_for_orientation").as_bool();
    
    // 订阅者
    rtk_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/gps", 10,
      std::bind(&ComplementaryFilterNode::rtk_callback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&ComplementaryFilterNode::odom_callback, this, std::placeholders::_1));
    
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10,
      std::bind(&ComplementaryFilterNode::imu_callback, this, std::placeholders::_1));
    
    // 发布者
    fused_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/fused/odom", 10);
    fused_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/fused/pose", 10);
    
    // TF广播器
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // 初始化四元数
    current_pose_.orientation.w = 1.0;
    
    RCLCPP_INFO(this->get_logger(), "Complementary Filter Node initialized");
    RCLCPP_INFO(this->get_logger(), "Position alpha: %.3f, Orientation alpha: %.3f", 
                alpha_position_, alpha_orientation_);
    RCLCPP_INFO(this->get_logger(), "Use IMU for orientation: %s", 
                use_imu_for_orientation_ ? "true" : "false");
  }

private:
  void rtk_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // 如果是第一次收到RTK数据，用它来初始化融合位姿
    if (!is_rtk_init_) {
      current_pose_.position = msg->pose.pose.position;
      if (!use_imu_for_orientation_) {
        current_pose_.orientation = msg->pose.pose.orientation;
      }
      is_rtk_init_ = true;
      RCLCPP_INFO(this->get_logger(), "RTK initialized the fused pose.");
      return;
    }
    
    // 低通滤波：将当前融合位姿向RTK位姿缓慢拉近
    current_pose_.position.x = alpha_position_ * msg->pose.pose.position.x + 
                              (1 - alpha_position_) * current_pose_.position.x;
    current_pose_.position.y = alpha_position_ * msg->pose.pose.position.y + 
                              (1 - alpha_position_) * current_pose_.position.y;
    current_pose_.position.z = alpha_position_ * msg->pose.pose.position.z + 
                              (1 - alpha_position_) * current_pose_.position.z;
    
    // 如果不用IMU，对朝向也进行滤波
    if (!use_imu_for_orientation_) {
      // 使用四元数球面线性插值(SLERP)进行朝向滤波
      tf2::Quaternion rtk_quat, current_quat, filtered_quat;
      tf2::fromMsg(msg->pose.pose.orientation, rtk_quat);
      tf2::fromMsg(current_pose_.orientation, current_quat);
      
      // 确保四元数方向一致
      if (rtk_quat.dot(current_quat) < 0.0) {
        // 修复：明确使用成员函数形式的负号运算符
        rtk_quat = rtk_quat.operator-();
      }
      
      filtered_quat = current_quat.slerp(rtk_quat, alpha_orientation_);
      current_pose_.orientation = tf2::toMsg(filtered_quat);
    }
    
    publish_fused_data(msg->header.stamp);
  }
  
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!is_rtk_init_) {
      // 没有RTK初始位姿前，无法进行融合
      return;
    }
    
    if (!last_odom_) {
      // 第一次收到里程计数据，只记录，不计算增量
      last_odom_ = msg;
      last_odom_time_ = msg->header.stamp;
      return;
    }
    
    // 计算自上一次里程计更新以来的时间差
    rclcpp::Time current_time = msg->header.stamp;
    rclcpp::Time last_time = last_odom_time_;
    double dt = (current_time - last_time).seconds();
    
    if (dt <= 0) {
      return;
    }
    
    // 获取Twist中的线速度来计算增量
    double delta_x = msg->twist.twist.linear.x * dt;
    double delta_y = msg->twist.twist.linear.y * dt;
    double delta_z = msg->twist.twist.linear.z * dt;
    
    // 高通滤波：将里程计的增量直接累加到融合位姿上
    current_pose_.position.x += delta_x;
    current_pose_.position.y += delta_y;
    current_pose_.position.z += delta_z;
    
    // 如果不使用IMU，使用里程计的方向
    if (!use_imu_for_orientation_) {
      current_pose_.orientation = msg->pose.pose.orientation;
    }
    
    publish_fused_data(msg->header.stamp);
    
    // 更新上一次的里程计数据和时间
    last_odom_ = msg;
    last_odom_time_ = msg->header.stamp;
  }
  
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!use_imu_for_orientation_) {
      return; // 如果不使用IMU进行朝向估计，则直接返回
    }
    
    if (!is_imu_init_) {
      // 第一次收到IMU数据，直接使用
      current_pose_.orientation = msg->orientation;
      is_imu_init_ = true;
      RCLCPP_INFO(this->get_logger(), "IMU initialized the orientation.");
      return;
    }
    
    // 对朝向进行互补滤波
    // 使用四元数球面线性插值(SLERP)进行朝向滤波
    tf2::Quaternion imu_quat, current_quat, filtered_quat;
    tf2::fromMsg(msg->orientation, imu_quat);
    tf2::fromMsg(current_pose_.orientation, current_quat);
    
    // 确保四元数方向一致
    if (imu_quat.dot(current_quat) < 0.0) {
      // 修复：明确使用成员函数形式的负号运算符
      imu_quat = imu_quat.operator-();
    }
    
    filtered_quat = current_quat.slerp(imu_quat, alpha_orientation_);
    current_pose_.orientation = tf2::toMsg(filtered_quat);
    
    // 记录角速度，用于发布
    last_angular_velocity_ = msg->angular_velocity;
    
    publish_fused_data(msg->header.stamp);
  }
  
  void publish_fused_data(const builtin_interfaces::msg::Time & stamp)
  {
    if (!is_rtk_init_) {
      return;
    }
    
    // 1. 发布Odometry消息
    auto fused_odom = std::make_unique<nav_msgs::msg::Odometry>();
    fused_odom->header.stamp = stamp;
    fused_odom->header.frame_id = "odom";
    fused_odom->child_frame_id = "base_link";
    fused_odom->pose.pose = current_pose_;
    
    // 填充Twist信息
    if (last_odom_) {
      fused_odom->twist.twist.linear = last_odom_->twist.twist.linear;
    }
    
    if (last_angular_velocity_) {
      fused_odom->twist.twist.angular = last_angular_velocity_.value();
    }
    
    fused_odom_pub_->publish(std::move(fused_odom));
    
    // 2. 发布PoseStamped消息
    auto fused_pose = std::make_unique<geometry_msgs::msg::PoseStamped>();
    fused_pose->header = fused_odom->header;
    fused_pose->pose = current_pose_;
    fused_pose_pub_->publish(std::move(fused_pose));
    
    // 3. 发布TF变换: odom -> base_link
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    t.transform.translation.x = current_pose_.position.x;
    t.transform.translation.y = current_pose_.position.y;
    t.transform.translation.z = current_pose_.position.z;
    t.transform.rotation = current_pose_.orientation;
    tf_broadcaster_->sendTransform(t);
  }
  
  // 参数
  double alpha_position_;      // 位置滤波系数
  double alpha_orientation_;   // 朝向滤波系数
  bool use_imu_for_orientation_; // 是否使用IMU进行朝向估计
  
  // 订阅者
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr rtk_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  
  // 发布者
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr fused_pose_pub_;
  
  // TF广播器
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  // 内部状态变量
  geometry_msgs::msg::Pose current_pose_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  builtin_interfaces::msg::Time last_odom_time_;
  std::optional<geometry_msgs::msg::Vector3> last_angular_velocity_;
  bool is_rtk_init_ = false;
  bool is_imu_init_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ComplementaryFilterNode>());
  rclcpp::shutdown();
  return 0;
}