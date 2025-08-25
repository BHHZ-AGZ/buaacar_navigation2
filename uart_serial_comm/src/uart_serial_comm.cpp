#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <serial/serial.h>
#include <vector>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <thread>
#include <cstring>
#include <cmath>

// 计算校验和函数
uint8_t calculateChecksum(const uint8_t* data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; ++i) {
        checksum += data[i];
    }
    return checksum;
}

class UartSerialComm : public rclcpp::Node {
public:
    UartSerialComm() : Node("uart_serial_comm") {
        // 声明并获取参数
        this->declare_parameter("publish_tf", false);
        this->declare_parameter("base_frame_id", "base_link");
        this->declare_parameter("ultrasound_count", 8);  // 修改为8个传感器
        this->get_parameter("publish_tf", publish_tf_);
        this->get_parameter("base_frame_id", base_frame_id_);
        this->get_parameter("ultrasound_count", ultrasound_count_);
        
        // 初始化里程计参数
        wheel_diameter_ = 0.170;
        wheel_radius_ = wheel_diameter_ / 2.0;
        wheel_base_ = 0.42920;
        
        // 初始化里程计状态
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        
        // 初始化时间戳
        last_time_ = this->get_clock()->now();
        
        // 串口初始化
        try {
            serial_port_.setPort("/dev/stm32_port");
            serial_port_.setBaudrate(9600);
            serial_port_.setBytesize(serial::eightbits);
            serial_port_.setStopbits(serial::stopbits_one);
            serial_port_.setParity(serial::parity_none);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            serial_port_.setTimeout(to);
            serial_port_.open();
        } catch (serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open serial port: %s", e.what());
        }

        if (serial_port_.isOpen()) {
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
            start_receive_thread();
        }

        // 创建速度指令订阅者 /cmd_vel_nav
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                if (serial_port_.isOpen()) {
                    // 1. 转换速度指令（保留原有缩放逻辑）
                    int16_t linear_x = static_cast<int16_t>(msg->linear.x * 10);  // 线速度放大10倍
                    int16_t angular_z = static_cast<int16_t>(msg->angular.z * 10);  // 角速度放大10倍

                    // 2. 构建带帧同步的数据包
                    // 数据包结构：帧头1(0xAA) + 帧头2(0x55) + 线速度高8位 + 线速度低8位 + 角速度高8位 + 角速度低8位 + 校验和 + 帧尾(0xBB)
                    std::vector<uint8_t> packet;
                    packet.push_back(0xAA);  // 帧头1：固定标识
                    packet.push_back(0x55);  // 帧头2：固定标识
                    
                    // 线速度拆分（高8位 + 低8位）
                    packet.push_back(static_cast<uint8_t>(linear_x >> 8));
                    packet.push_back(static_cast<uint8_t>(linear_x & 0xFF));
                    
                    // 角速度拆分（高8位 + 低8位）
                    packet.push_back(static_cast<uint8_t>(angular_z >> 8));
                    packet.push_back(static_cast<uint8_t>(angular_z & 0xFF));
                    
                    // 校验和（仅计算数据部分：线速度2字节 + 角速度2字节）
                    uint8_t checksum = calculateChecksum(packet.data() + 2, 4);  // 跳过帧头
                    packet.push_back(checksum);
                    
                    packet.push_back(0xBB);  // 帧尾：固定标识

                    // 3. 打印发送的数据包（调试用）
                    // RCLCPP_DEBUG(this->get_logger(), "发送速度指令数据包:");
                    // for (size_t i = 0; i < packet.size(); ++i) {
                    //     RCLCPP_DEBUG(this->get_logger(), "  字节%d: 0x%02X", static_cast<int>(i), packet[i]);
                    // }

                    // 4. 一次性发送整个数据包（避免时序问题）
                    try {
                        serial_port_.write(packet);  // 批量发送，替代逐个字节发送
                        // 短延迟确保数据发送完成（根据波特率调整，9600波特率下1字节约1ms）
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    } catch (serial::IOException& e) {
                        RCLCPP_ERROR(this->get_logger(), "串口发送错误: %s", e.what());
                    }
                }
            });

        // 创建里程计发布者
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        
        // 创建超声波数据发布者
        ultrasound_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/ultrasound_data", 10);
        RCLCPP_INFO(this->get_logger(), "超声波发布者创建成功: /ultrasound_data (支持%d个传感器)", ultrasound_count_);
        
        // 初始化TF广播器
        if (publish_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }
    }

    ~UartSerialComm() {
        if (receive_thread_.joinable()) {
            keep_receiving_ = false;
            receive_thread_.join();
        }
        if (serial_port_.isOpen()) {
            serial_port_.close();
        }
    }

private:
    // ROS相关成员
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr ultrasound_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // 参数配置
    bool publish_tf_;
    std::string base_frame_id_;
    int ultrasound_count_;  // 传感器数量（8个）
    
    // 串口相关成员
    serial::Serial serial_port_;
    std::thread receive_thread_;
    bool keep_receiving_ = false;
    
    // 里程计帧头和状态
    const uint8_t odom_frame_header_[4] = {0x43, 0x6C, 0x60, 0x03};
    int odom_header_state_ = 0;
    
    // 超声波帧头和状态（修改为10字节帧：2字节头+8字节数据）
    const uint8_t ultrasound_frame_header_[2] = {0xAB, 0xCD};  // 前2位固定帧头
    int ultrasound_header_state_ = 0;
    const size_t ULTRASOUND_DATA_SIZE = 8;  // 后8位为传感器数据
    uint8_t ultrasound_data_[8] = {0};      // 8个传感器数据缓冲区
    int ultrasound_data_count_ = 0;
    
    // 里程计参数
    double wheel_diameter_;
    double wheel_radius_;
    double wheel_base_;
    
    // 里程计状态
    double x_;
    double y_;
    double theta_;
    rclcpp::Time last_time_;

    void start_receive_thread() {
        keep_receiving_ = true;
        receive_thread_ = std::thread(&UartSerialComm::receive_data, this);
    }

    void receive_data() {
        const size_t ODOM_DATA_SIZE = 4;
        uint8_t odom_data_[ODOM_DATA_SIZE] = {0};
        int odom_data_count_ = 0;

        while (keep_receiving_ && rclcpp::ok()) {
            if (serial_port_.isOpen() && serial_port_.available() >= 1) {
                uint8_t current_byte;
                try {
                    if (serial_port_.read(&current_byte, 1) != 1) {
                        continue;
                    }

                    // 优先检测超声波帧头（0xAB, 0xCD）
                    if (ultrasound_header_state_ < 2) {
                        if (current_byte == ultrasound_frame_header_[ultrasound_header_state_]) {
                            ultrasound_header_state_++;
                            if (ultrasound_header_state_ == 2) {
                                ultrasound_data_count_ = 0;  // 帧头匹配完成，开始接收8字节数据
                                RCLCPP_DEBUG(this->get_logger(), "超声波帧头匹配成功，准备接收8字节数据");
                            }
                        } else {
                            ultrasound_header_state_ = 0;  // 帧头匹配失败，重置
                        }
                    }
                    // 超声波帧头匹配成功，接收8字节数据
                    else {
                        ultrasound_data_[ultrasound_data_count_] = current_byte;
                        ultrasound_data_count_++;
                        
                        // 8字节数据接收完成
                        if (static_cast<size_t>(ultrasound_data_count_) >= ULTRASOUND_DATA_SIZE) {
                            parse_ultrasound_data(ultrasound_data_);
                            // 重置状态，准备下一帧
                            ultrasound_header_state_ = 0;
                            ultrasound_data_count_ = 0;
                        }
                        continue;
                    }

                    // 检测里程计帧头
                    if (odom_header_state_ < 4) {
                        if (current_byte == odom_frame_header_[odom_header_state_]) {
                            odom_header_state_++;
                            if (odom_header_state_ == 4) {
                                odom_data_count_ = 0;
                                RCLCPP_DEBUG(this->get_logger(), "里程计帧头匹配成功");
                            }
                        } else {
                            odom_header_state_ = (current_byte == odom_frame_header_[0]) ? 1 : 0;
                        }
                    }
                    // 里程计帧头匹配成功，接收数据部分
                    else {
                        odom_data_[odom_data_count_] = current_byte;
                        odom_data_count_++;
                        
                        if (static_cast<size_t>(odom_data_count_) >= ODOM_DATA_SIZE) {
                            parse_odom_data(odom_data_);
                            odom_header_state_ = 0;
                            odom_data_count_ = 0;
                        }
                    }

                } catch (serial::IOException& e) {
                    RCLCPP_ERROR(this->get_logger(), "串口读取错误: %s", e.what());
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    // 解析里程计数据
    void parse_odom_data(const uint8_t* data) {
        int16_t left_raw = static_cast<int16_t>((data[1] << 8) | data[0]);
        float left_rpm = left_raw * 0.1f;
        
        int16_t right_raw = static_cast<int16_t>((data[3] << 8) | data[2]);
        float right_rpm = -right_raw * 0.1f;

        calculate_odometry(left_rpm, right_rpm);
    }

    // 解析超声波数据（修改为8个传感器）
    void parse_ultrasound_data(const uint8_t* data) {
        std::vector<float> distances;

        // 解析8个传感器数据（每个1字节，单位：厘米，转换为米）
        // 假设每个字节代表距离值（0-255厘米）
        for (int i = 0; i < 8; ++i) {
            uint8_t distance_cm = data[i];  // 1字节数据（厘米）
            float distance_m = distance_cm / 100.0f;  // 转换为米
            distances.push_back(distance_m);
        }

        // 打印8个传感器数据
        // RCLCPP_INFO(this->get_logger(), "超声波数据（8个传感器）:");
        // for (size_t i = 0; i < distances.size(); ++i) {
        //     RCLCPP_INFO(this->get_logger(), "  传感器%d: %.2f米 (原始值: 0x%02X)", 
        //               static_cast<int>(i), distances[i], data[i]);
        // }

        // 发布8个传感器数据
        std_msgs::msg::Float32MultiArray msg;
        msg.data = distances;
        ultrasound_pub_->publish(msg);
    }

    // 计算里程计数据并发布
    void calculate_odometry(float left_rpm, float right_rpm) {
        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        if (dt <= 0.0) {
            return;
        }

        double left_vel = (left_rpm * 2 * M_PI * wheel_radius_) / 60.0;
        double right_vel = (right_rpm * 2 * M_PI * wheel_radius_) / 60.0;

        double linear_vel = (left_vel + right_vel) / 2.0;
        double angular_vel = (right_vel - left_vel) / wheel_base_;

        double delta_x = linear_vel * cos(theta_) * dt;
        double delta_y = linear_vel * sin(theta_) * dt;
        double delta_theta = angular_vel * dt;

        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_theta;

        theta_ = atan2(sin(theta_), cos(theta_));

        if (publish_tf_ && tf_broadcaster_) {
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped.header.stamp = current_time;
            transform_stamped.header.frame_id = "odom";
            transform_stamped.child_frame_id = base_frame_id_;

            transform_stamped.transform.translation.x = x_;
            transform_stamped.transform.translation.y = y_;
            transform_stamped.transform.translation.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, theta_);
            transform_stamped.transform.rotation.x = q.x();
            transform_stamped.transform.rotation.y = q.y();
            transform_stamped.transform.rotation.z = q.z();
            transform_stamped.transform.rotation.w = q.w();

            tf_broadcaster_->sendTransform(transform_stamped);
        }

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = base_frame_id_;

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.x = 0.0;
        odom.pose.pose.orientation.y = 0.0;
        odom.pose.pose.orientation.z = sin(theta_ / 2.0);
        odom.pose.pose.orientation.w = cos(theta_ / 2.0);

        odom.twist.twist.linear.x = linear_vel;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = angular_vel;

        odom_pub_->publish(odom);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UartSerialComm>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
    