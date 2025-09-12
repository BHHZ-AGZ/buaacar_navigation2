#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
// 新增：引入Range消息头文件
#include <sensor_msgs/msg/range.hpp>
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
        this->declare_parameter("ultrasound_count", 8);  // 8个传感器
        // 新增：声明超声波传感器相关参数（按实际硬件修改）
        this->declare_parameter("ultrasound_frame_prefix", "ultrasound_link");  // 传感器坐标系前缀
        this->declare_parameter("ultrasound_min_range", 0.21);  // 最小探测距离（2cm）
        this->declare_parameter("ultrasound_max_range", 2.5);   // 最大探测距离（2m）
        this->declare_parameter("ultrasound_fov", 0.79);       // 视野角（20°，转弧度：20*M_PI/180≈0.349）
        
        // 获取原有参数
        this->get_parameter("publish_tf", publish_tf_);
        this->get_parameter("base_frame_id", base_frame_id_);
        this->get_parameter("ultrasound_count", ultrasound_count_);
        // 新增：获取超声波参数
        this->get_parameter("ultrasound_frame_prefix", ultrasound_frame_prefix_);
        this->get_parameter("ultrasound_min_range", ultrasound_min_range_);
        this->get_parameter("ultrasound_max_range", ultrasound_max_range_);
        this->get_parameter("ultrasound_fov", ultrasound_fov_);
        
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

        // 创建速度指令订阅者 /cmd_vel_nav（原有逻辑不变）
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                if (serial_port_.isOpen()) {
                    int16_t linear_x = static_cast<int16_t>(msg->linear.x * 10);  
                    int16_t angular_z = static_cast<int16_t>(msg->angular.z * 10);  

                    std::vector<uint8_t> packet;
                    packet.push_back(0xAA);  
                    packet.push_back(0x55);  
                    
                    packet.push_back(static_cast<uint8_t>(linear_x >> 8));
                    packet.push_back(static_cast<uint8_t>(linear_x & 0xFF));
                    
                    packet.push_back(static_cast<uint8_t>(angular_z >> 8));
                    packet.push_back(static_cast<uint8_t>(angular_z & 0xFF));
                    
                    uint8_t checksum = calculateChecksum(packet.data() + 2, 4);  
                    packet.push_back(checksum);
                    
                    packet.push_back(0xBB);  

                    try {
                        serial_port_.write(packet);  
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    } catch (serial::IOException& e) {
                        RCLCPP_ERROR(this->get_logger(), "串口发送错误: %s", e.what());
                    }
                }
            });

        // 创建里程计发布者（原有逻辑不变）
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        
        // 创建超声波Float32MultiArray发布者（原有逻辑不变）
        ultrasound_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/ultrasound_data", 10);
        
        // 新增：创建8个Range类型发布者（对应8个传感器）
        ultrasound_range_pubs_.resize(ultrasound_count_);
        for (int i = 0; i < ultrasound_count_; ++i) {
            // 话题名格式：/ultrasound_range/ultrasound_0 ~ /ultrasound_range/ultrasound_7
            std::string topic_name = "ultrasound_" + std::to_string(i);
            ultrasound_range_pubs_[i] = this->create_publisher<sensor_msgs::msg::Range>(
                topic_name, 10);
            RCLCPP_INFO(this->get_logger(), "超声波Range发布者创建成功: %s", topic_name.c_str());
        }
        
        // 初始化TF广播器（原有逻辑不变）
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
    // ROS相关成员（原有成员不变，新增Range发布者容器）
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr ultrasound_pub_;
    // 新增：存储8个Range发布者的容器
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr> ultrasound_range_pubs_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // 参数配置（原有参数不变，新增超声波Range相关参数）
    bool publish_tf_;
    std::string base_frame_id_;
    int ultrasound_count_;  
    // 新增：超声波Range参数
    std::string ultrasound_frame_prefix_;  // 传感器坐标系前缀（如"ultrasound_"）
    double ultrasound_min_range_;          // 最小探测距离（m）
    double ultrasound_max_range_;          // 最大探测距离（m）
    double ultrasound_fov_;                // 视野角（rad）
    
    // 串口相关成员（原有不变）
    serial::Serial serial_port_;
    std::thread receive_thread_;
    bool keep_receiving_ = false;
    
    // 里程计帧头和状态（原有不变）
    const uint8_t odom_frame_header_[4] = {0x43, 0x6C, 0x60, 0x03};
    int odom_header_state_ = 0;
    
    // 超声波帧头和状态（原有不变）
    const uint8_t ultrasound_frame_header_[2] = {0xAB, 0xCD};  
    int ultrasound_header_state_ = 0;
    const size_t ULTRASOUND_DATA_SIZE = 8;  
    uint8_t ultrasound_data_[8] = {0};      
    int ultrasound_data_count_ = 0;
    
    // 里程计参数和状态（原有不变）
    double wheel_diameter_;
    double wheel_radius_;
    double wheel_base_;
    double x_;
    double y_;
    double theta_;
    rclcpp::Time last_time_;

    // 原有函数：启动接收线程（逻辑不变）
    void start_receive_thread() {
        keep_receiving_ = true;
        receive_thread_ = std::thread(&UartSerialComm::receive_data, this);
    }

    // 原有函数：接收串口数据（逻辑不变）
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

                    // 优先检测超声波帧头（原有逻辑不变）
                    if (ultrasound_header_state_ < 2) {
                        if (current_byte == ultrasound_frame_header_[ultrasound_header_state_]) {
                            ultrasound_header_state_++;
                            if (ultrasound_header_state_ == 2) {
                                ultrasound_data_count_ = 0;  
                                RCLCPP_DEBUG(this->get_logger(), "超声波帧头匹配成功，准备接收8字节数据");
                            }
                        } else {
                            ultrasound_header_state_ = 0;  
                        }
                    }
                    // 超声波帧头匹配成功，接收8字节数据（原有逻辑不变）
                    else {
                        ultrasound_data_[ultrasound_data_count_] = current_byte;
                        ultrasound_data_count_++;
                        
                        if (static_cast<size_t>(ultrasound_data_count_) >= ULTRASOUND_DATA_SIZE) {
                            parse_ultrasound_data(ultrasound_data_);  // 解析数据（内部新增Range发布）
                            ultrasound_header_state_ = 0;
                            ultrasound_data_count_ = 0;
                        }
                        continue;
                    }

                    // 检测里程计帧头（原有逻辑不变）
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
                    // 里程计帧头匹配成功，接收数据部分（原有逻辑不变）
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

    // 原有函数：解析里程计数据（逻辑不变）
    void parse_odom_data(const uint8_t* data) {
        int16_t left_raw = static_cast<int16_t>((data[1] << 8) | data[0]);
        float left_rpm = left_raw * 0.1f;
        
        int16_t right_raw = static_cast<int16_t>((data[3] << 8) | data[2]);
        float right_rpm = -right_raw * 0.1f;

        calculate_odometry(left_rpm, right_rpm);
    }

    // 原有函数：解析超声波数据（新增Range消息发布逻辑）
    void parse_ultrasound_data(const uint8_t* data) {
        std::vector<float> distances;
    for (int i = 0; i < 8; ++i) {
        uint8_t distance_cm = data[i];  
        float distance_m = distance_cm / 100.0f;  
        distances.push_back(distance_m);  // 原始顺序存入distances（不修改，保持与/ultrasound_data一致）
    }
//数据调整
    std::unordered_map<int, int> index_map = {
        {0, 1}, 
        {1, 0},
        {2, 7},  
        {3, 6}, 
        {4, 5},  
        {5, 4},  
        {6, 3},  
        {7, 2}   
    };

    // 新增：按映射表发布Range消息（遍历映射表，而非原始索引）
    for (auto& [original_idx, target_idx] : index_map) {
        // 确保目标索引在发布者数组范围内
        if (target_idx < 0 || target_idx >= ultrasound_range_pubs_.size()) {
            RCLCPP_WARN(this->get_logger(), "无效的传感器索引：%d，跳过发布", target_idx);
            continue;
        }
        // 仅当发布者有订阅者时发布（避免无效数据）
        if (ultrasound_range_pubs_[target_idx]->get_subscription_count() == 0) {
            continue;
        }

        // 获取调整后的数据（原始索引对应的数据 → 目标传感器）
        float adjusted_distance = distances[original_idx];
        auto range_msg = sensor_msgs::msg::Range();

        // 1. 消息头部（时间戳+坐标系：ultrasound_1~ultrasound_8，对应target_idx 0~7）
        range_msg.header.stamp = this->get_clock()->now();
        range_msg.header.frame_id = ultrasound_frame_prefix_ + std::to_string(target_idx + 1);  // 如"ultrasound_1"

        // 2. 传感器类型（超声波固定为0）
        range_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;

        // 3. 视野角、最小/最大量程（从参数获取）
        range_msg.field_of_view = ultrasound_fov_;
        range_msg.min_range = ultrasound_min_range_;
        range_msg.max_range = ultrasound_max_range_;

        // 4. 实际测量距离（过滤无效值：小于最小量程或大于最大量程设为inf）
        if (adjusted_distance < ultrasound_min_range_ || adjusted_distance > ultrasound_max_range_) {
            range_msg.range = std::numeric_limits<float>::infinity();
        } else {
            range_msg.range = adjusted_distance;
        }

        // 5. 发布Range消息（目标传感器的发布者）
        ultrasound_range_pubs_[target_idx]->publish(range_msg);
    }

        // 原有逻辑：发布Float32MultiArray消息（不变）
        std_msgs::msg::Float32MultiArray msg;
        msg.data = distances;
        ultrasound_pub_->publish(msg);
    }

    // 原有函数：计算里程计数据并发布（逻辑不变）
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
