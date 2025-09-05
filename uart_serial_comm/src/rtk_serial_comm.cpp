#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion.hpp>  
#include <geometry_msgs/msg/vector3.hpp>  
#include <vector>
#include <chrono>
#include <thread>
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>

// 辅助函数：删除字符串中所有空白字符
void remove_all_spaces(std::string& s) {
    s.erase(std::remove_if(s.begin(), s.end(), [](unsigned char c) {
        return std::isspace(c) != 0;
    }), s.end());
}

// 辅助函数：分割字符串
std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        if (!token.empty()) {
            tokens.push_back(token);
        }
    }
    return tokens;
}

class RTKImuNavPublisher : public rclcpp::Node {
public:
    RTKImuNavPublisher() : Node("rtk_imu_nav_publisher") {
        // 声明参数（仅参数名+默认值，兼容ROS 2 Humble）
        this->declare_parameter<std::string>("port", "/dev/RTK_port");
        this->declare_parameter<int>("baud", 115200);
        this->declare_parameter<std::string>("gps_frame_id", "gps_link");
        this->declare_parameter<std::string>("imu_frame_id", "gyro_link");
        this->declare_parameter<double>("gyro_scale", 0.0174533);
        this->declare_parameter<double>("acc_scale", 0.00980665);
        this->declare_parameter<int>("buf_max_size", 1000);

        // 获取参数
        port_ = this->get_parameter("port").as_string();
        baud_ = this->get_parameter("baud").as_int();
        gps_frame_id_ = this->get_parameter("gps_frame_id").as_string();
        imu_frame_id_ = this->get_parameter("imu_frame_id").as_string();
        gyro_scale_ = this->get_parameter("gyro_scale").as_double();
        acc_scale_ = this->get_parameter("acc_scale").as_double();
        buf_max_size_ = this->get_parameter("buf_max_size").as_int();

        // 初始化串口
        if (init_serial()) {
            RCLCPP_INFO(this->get_logger(), "RTK串口初始化成功！端口: %s, 波特率: %d", port_.c_str(), baud_);
            // 创建发布者
            gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
            imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
            // 启动接收线程
            receive_thread_ = std::thread(&RTKImuNavPublisher::receive_loop, this);
            receive_thread_.detach();
        } else {
            RCLCPP_FATAL(this->get_logger(), "RTK串口初始化失败！请检查端口权限或设备连接");
            rclcpp::shutdown();
        }
    }

    ~RTKImuNavPublisher() {
        running_ = false;
        if (serial_.isOpen()) serial_.close();
    }

private:
    // 串口与参数成员
    serial::Serial serial_;
    std::thread receive_thread_;
    bool running_ = true;
    std::string buf_;
    std::string port_;
    int baud_;
    std::string gps_frame_id_;
    std::string imu_frame_id_;
    double gyro_scale_;
    double acc_scale_;
    int buf_max_size_;

    // 发布者
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;


    bool init_serial() {
        try {
            serial_.setPort(port_);
            serial_.setBaudrate(baud_);
            serial_.setBytesize(serial::eightbits);
            serial_.setStopbits(serial::stopbits_one);
            serial_.setParity(serial::parity_none);
            serial_.setFlowcontrol(serial::flowcontrol_none);
            
            // 修复：创建左值对象，再传入setTimeout
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_.setTimeout(timeout);
            
            serial_.open();
            return serial_.isOpen();
        } catch (serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "串口错误: %s", e.what());
            return false;
        }
    }


    void receive_loop() {
        while (running_ && rclcpp::ok()) {
            if (serial_.isOpen() && serial_.available() > 0) {
                buf_ += serial_.read(serial_.available());
                parse_and_publish_gps();
                parse_and_publish_imu();
                // 修复：强制转换为unsigned long，避免签名不匹配警告
                if (buf_.size() > static_cast<size_t>(buf_max_size_)) {
                    buf_ = buf_.substr(buf_.size() - static_cast<size_t>(buf_max_size_));
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    void parse_and_publish_gps() {
        const std::string gps_prefix = "CloudTrace5660,POS:";
        size_t gps_start = buf_.find(gps_prefix);

        while (gps_start != std::string::npos) {
            size_t gps_end = buf_.find(gps_prefix, gps_start + gps_prefix.size());
            if (gps_end == std::string::npos) gps_end = buf_.size();

            std::string gps_data = buf_.substr(gps_start + gps_prefix.size(), gps_end - gps_start - gps_prefix.size());
            remove_all_spaces(gps_data);
            std::vector<std::string> parts = split(gps_data, ',');

            // 修复1：parts.size()是unsigned long，用%lu格式符
            if (parts.size() < 10) {
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "GPS字段不足（需10个，实际%lu），原始数据：%s", parts.size(), gps_data.c_str());
                gps_start = buf_.find(gps_prefix, gps_end);
                continue;
            }

            try {
                // 修复2：time和speed变量未使用，添加注释说明（或删除）
                [[maybe_unused]] double time = std::stod(parts[0]);          // 1. 时间
                double latitude = std::stod(parts[1]);                      // 2. 纬度
                double longitude = std::stod(parts[2]);                     // 3. 经度
                int fix_quality = std::stoi(parts[3]);                      // 4. 定位级别
                int sat_count = std::stoi(parts[4]);                        // 5. 卫星数量
                double hdop = std::stod(parts[5]);                          // 6. 水平误差
                double altitude = std::stod(parts[6]);                      // 7. 高度
                [[maybe_unused]] double speed = std::stod(parts[7]);         // 8. 速度
                char16_t status;                           // 9. 状态


                // 构建NavSatFix消息
                sensor_msgs::msg::NavSatFix msg;
                msg.header.stamp = this->now();
                msg.header.frame_id = gps_frame_id_;
                msg.latitude = latitude;
                msg.longitude = longitude;
                msg.altitude = altitude;
                double horizontal_error;
                double vorizontal_error;
                // 定位状态映射
                if (fix_quality == 5) {
                    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
                    horizontal_error = hdop * 0.2;
                } else if (fix_quality == 4) {
                    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
                    horizontal_error = hdop * 0.02;
                }else if(fix_quality == 2){
                    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
                    horizontal_error = hdop * 0.7;
                } else {
                    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
                    horizontal_error = 1000000;
                }
                msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;
                vorizontal_error = 
                // 协方差计算
                msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;   //近似估计协方差
                double hdop_sq = horizontal_error * horizontal_error / 2;
                double vdop_sq = hdop_sq * 3;
                msg.position_covariance = {
                    hdop_sq, 0.0,    0.0,
                    0.0,    hdop_sq, 0.0,
                    0.0,    0.0,    vdop_sq
                };

                // 发布消息
                gps_pub_->publish(msg);
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "GPS解析成功（status='A'）：\n"
                    "  纬度：%.8f, 经度：%.8f\n"
                    "  定位级别：%d, 卫星数：%d, 水平误差：%.1f\n"
                    "  高度：%.1f m",
                    latitude, longitude, fix_quality, sat_count, hdop, altitude);

            } catch (std::invalid_argument& e) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "GPS字段类型错误（非数字）：%s，原始数据：%s", e.what(), gps_data.c_str());
            }catch (std::out_of_range& e) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "GPS字段超出范围：%s，原始数据：%s", e.what(), gps_data.c_str());
            }

            gps_start = buf_.find(gps_prefix, gps_end);
        }
    }


    void parse_and_publish_imu() {
        size_t imu_start = buf_.find("MAG:");
        if (imu_start == std::string::npos) imu_start = buf_.find("GYRO:");

        while (imu_start != std::string::npos) {
            size_t imu_end = buf_.find("CloudTrace5660,POS:", imu_start);
            if (imu_end == std::string::npos) imu_end = buf_.size();

            std::string imu_data = buf_.substr(imu_start, imu_end - imu_start);
            remove_all_spaces(imu_data);
            std::vector<std::string> parts = split(imu_data, ',');

            int mag[3] = {0}, gyro[3] = {0}, acc[3] = {0};
            bool has_mag = false, has_gyro = false, has_acc = false;

            // 解析MAG/GYRO/ACC
            for (size_t i = 0; i < parts.size(); ++i) {
                if (parts[i].substr(0, 4) == "MAG:" && i + 2 < parts.size()) {
                    try {
                        mag[0] = std::stoi(parts[i].substr(4));
                        mag[1] = std::stoi(parts[i + 1]);
                        mag[2] = std::stoi(parts[i + 2]);
                        has_mag = true;
                    } catch (...) { RCLCPP_DEBUG(this->get_logger(), "MAG解析失败"); }
                } else if (parts[i].substr(0, 5) == "GYRO:" && i + 2 < parts.size()) {
                    try {
                        gyro[0] = std::stoi(parts[i].substr(5));
                        gyro[1] = std::stoi(parts[i + 1]);
                        gyro[2] = std::stoi(parts[i + 2]);
                        has_gyro = true;
                    } catch (...) { RCLCPP_DEBUG(this->get_logger(), "GYRO解析失败"); }
                } else if (parts[i].substr(0, 4) == "ACC:" && i + 2 < parts.size()) {
                    try {
                        acc[0] = std::stoi(parts[i].substr(4));
                        acc[1] = std::stoi(parts[i + 1]);
                        acc[2] = std::stoi(parts[i + 2]);
                        has_acc = true;
                    } catch (...) { RCLCPP_DEBUG(this->get_logger(), "ACC解析失败"); }
                }
            }

            // 发布IMU消息（需GYRO+ACC）
            if (has_gyro && has_acc) {
                sensor_msgs::msg::Imu msg;
                msg.header.stamp = this->now();
                msg.header.frame_id = imu_frame_id_;

                // 修复1：Quaternion结构体逐个赋值（不能用{}直接赋值）
                msg.orientation.x = 0.0;
                msg.orientation.y = 0.0;
                msg.orientation.z = 0.0;
                msg.orientation.w = 1.0;
                msg.orientation_covariance[0] = -1.0;

                // 修复2：角速度Vector3逐个赋值
                msg.angular_velocity.x = gyro[0] * gyro_scale_;
                msg.angular_velocity.y = gyro[1] * gyro_scale_;
                msg.angular_velocity.z = gyro[2] * gyro_scale_;
                msg.angular_velocity_covariance = {0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001};

                // 修复3：线加速度Vector3逐个赋值
                msg.linear_acceleration.x = acc[0] * acc_scale_;
                msg.linear_acceleration.y = acc[1] * acc_scale_;
                msg.linear_acceleration.z = acc[2] * acc_scale_;
                msg.linear_acceleration_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};

                imu_pub_->publish(msg);
                // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                //     "IMU解析成功：GYRO=(%.3f,%.3f,%.3f)rad/s, ACC=(%.3f,%.3f,%.3f)m/s², %sMAG",
                //     msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                //     msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                //     has_mag ? "" : "无");
            }

            // 跳到下一个IMU数据
            imu_start = buf_.find("MAG:", imu_end);
            if (imu_start == std::string::npos) imu_start = buf_.find("GYRO:", imu_end);
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RTKImuNavPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}