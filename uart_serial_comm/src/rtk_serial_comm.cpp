#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <chrono>
#include <thread>
#include <string>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <iomanip>

// 辅助函数：判断字符串是否为数字（整数/浮点数）
bool is_number(const std::string& s) {
    if (s.empty()) return false;
    // 允许数字、小数点、正负号
    return std::all_of(s.begin(), s.end(), [](char c) {
        return std::isdigit(c) || c == '.' || c == '-' || c == '+';
    });
}

// 辅助函数：删除字符串中所有空白字符（空格、制表符等）
void remove_all_spaces(std::string& s) {
    s.erase(std::remove_if(s.begin(), s.end(), [](unsigned char c) {
        return std::isspace(c) != 0;
    }), s.end());
}

// 辅助函数：彻底移除<break>标记及残留字符
void remove_break_tags(std::string& s) {
    const std::string break_tag = "<break>";
    size_t pos = 0;
    // 循环删除所有<break>标记
    while ((pos = s.find(break_tag, pos)) != std::string::npos) {
        s.erase(pos, break_tag.length());
    }
    // 清理残留的<和>字符（替换为逗号避免字段断裂）
    std::replace(s.begin(), s.end(), '<', ',');
    std::replace(s.begin(), s.end(), '>', ',');
}

// 辅助函数：移除时间戳（格式：[HH:MM:SS:mmm]）
void remove_timestamps(std::string& s) {
    size_t start = s.find('[');
    while (start != std::string::npos) {
        size_t end = s.find(']', start);
        if (end != std::string::npos) {
            // 删除整个时间戳（包括[]）
            s.erase(start, end - start + 1);
        } else {
            // 若没有闭合]，跳出循环避免死循环
            break;
        }
        // 继续查找下一个时间戳
        start = s.find('[', start);
    }
}

// 辅助函数：按指定分隔符分割字符串（忽略空字符串）
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

// 辅助函数：生成当前时间戳（格式：hhmmss.sss）
std::string get_current_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_tm = std::localtime(&now_time_t);
    
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch() % std::chrono::seconds(1)
    );
    
    std::stringstream ss;
    ss << std::setfill('0') 
       << std::setw(2) << now_tm->tm_hour    // 小时
       << std::setw(2) << now_tm->tm_min     // 分钟
       << std::setw(2) << now_tm->tm_sec     // 秒
       << "." 
       << std::setw(3) << ms.count();        // 毫秒
    
    return ss.str();
}

// 辅助函数：从数据中提取指定前缀的字段内容（指定结束标记避免跨字段）
std::string extract_field_content(const std::string& data, 
                                  const std::string& prefix, 
                                  const std::vector<std::string>& end_markers) {
    // 查找字段前缀（如"POS:"）
    size_t start = data.find(prefix);
    if (start == std::string::npos) {
        return ""; // 未找到前缀，返回空
    }
    // 从前缀后开始提取
    start += prefix.length();
    
    // 查找字段结束位置（取第一个结束标记的位置）
    size_t end = data.length();
    for (const auto& marker : end_markers) {
        size_t pos = data.find(marker, start);
        if (pos != std::string::npos && pos < end) {
            end = pos;
        }
    }
    
    // 返回提取的字段内容
    return data.substr(start, end - start);
}

// 辅助函数：清理并验证GPS的POS字段（严格输出9位，最后一位为状态位）
std::vector<std::string> process_gps_fields(const std::vector<std::string>& parts) {
    const size_t target_size = 9; // POS字段固定9位
    std::vector<std::string> processed;
    
    // 处理第0位：时间戳（如果原始数据为空或无效，使用当前时间戳）
    std::string time_field;
    if (parts.empty() || parts[0].empty() || !is_number(parts[0])) {
        time_field = get_current_timestamp();
        // RCLCPP_WARN(rclcpp::get_logger("rtk_imu_nav_publisher"), 
        //             "POS时间戳无效，使用当前时间戳: %s", time_field.c_str());
    } else {
        time_field = parts[0];
    }
    processed.push_back(time_field);
    
    // 处理第1-7位：其他数字字段
    for (size_t i = 1; i < 8; ++i) {
        std::string field;
        if (i < parts.size()) {
            field = parts[i];
            // 清理非数字字符
            field.erase(std::remove_if(field.begin(), field.end(), [](char c) {
                return !(std::isdigit(c) || c == '.' || c == '-' || c == '+');
            }), field.end());
        }
        
        processed.push_back(is_number(field) ? field : "0");
        if (!is_number(field)) {
            // RCLCPP_WARN(rclcpp::get_logger("rtk_imu_nav_publisher"), 
            //             "GPS字段[%zu]非有效数字，替换为0", i);
        }
    }
    
    // 处理第8位：状态位（如果不足9位，补充"v"）
    if (parts.size() >= 9) {
        std::string status = parts[8];
        if (status.empty() || !isalpha(status[0])) {
            processed.push_back("V");
        } else {
            processed.push_back(std::string(1, toupper(status[0])));
        }
    } else {
        // 字段不足9位，补充"v"
        processed.push_back("v");
        // RCLCPP_WARN(rclcpp::get_logger("rtk_imu_nav_publisher"), 
        //             "POS字段不足9位，补充状态位'v'");
    }
    
    return processed;
}

// 辅助函数：处理IMU字段（MAG/GYRO/ACC，固定4位：类型+3个测量值）
std::vector<std::string> process_imu_fields(const std::vector<std::string>& parts, 
                                            const std::string& type) {
    std::vector<std::string> processed;
    processed.push_back(type); // 第1位：字段类型（如"MAG"）
    
    // 处理第2-4位：测量值（数字）
    for (size_t i = 1; i < 4; ++i) {
        std::string field;
        if (i-1 < parts.size()) {
            field = parts[i-1];
            // 清理非数字字符（只保留数字和正负号）
            field.erase(std::remove_if(field.begin(), field.end(), [](char c) {
                return !(std::isdigit(c) || c == '-' || c == '+');
            }), field.end());
        }
        
        // 验证并添加字段
        if (is_number(field)) {
            processed.push_back(field);
        } else {
            processed.push_back("0");
            // RCLCPP_WARN(rclcpp::get_logger("rtk_imu_nav_publisher"), 
            //             "%s字段[%zu]非有效数字，替换为0", type.c_str(), i);
        }
    }
    
    return processed;
}

// 主节点类：处理串口通信、数据解析与消息发布
class RTKImuNavPublisher : public rclcpp::Node {
public:
    // 构造函数：初始化节点、参数与串口
    RTKImuNavPublisher() : Node("rtk_imu_nav_publisher") {
        // 1. 声明节点参数（带默认值）
        this->declare_parameter<std::string>("port", "/dev/RTK_port");       // 串口号
        this->declare_parameter<int>("baud", 115200);                        // 波特率
        this->declare_parameter<std::string>("gps_frame_id", "gps_link");    // GPS坐标系ID
        this->declare_parameter<std::string>("imu_frame_id", "gyro_link");   // IMU坐标系ID
        this->declare_parameter<double>("gyro_scale", 0.0174533);            // 陀螺仪度转弧度系数
        this->declare_parameter<double>("acc_scale", 0.00980665);            // 加速度单位转换系数
        this->declare_parameter<int>("buf_max_size", 8000);                  // 数据缓冲区最大长度

        // 2. 获取参数值
        port_ = this->get_parameter("port").as_string();
        baud_ = this->get_parameter("baud").as_int();
        gps_frame_id_ = this->get_parameter("gps_frame_id").as_string();
        imu_frame_id_ = this->get_parameter("imu_frame_id").as_string();
        gyro_scale_ = this->get_parameter("gyro_scale").as_double();
        acc_scale_ = this->get_parameter("acc_scale").as_double();
        buf_max_size_ = this->get_parameter("buf_max_size").as_int();

        // 3. 初始化串口并启动线程
        if (init_serial()) {
            RCLCPP_INFO(this->get_logger(), "RTK串口初始化成功！端口: %s, 波特率: %d", 
                       port_.c_str(), baud_);
            // 创建ROS消息发布者（队列大小10）
            gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
            imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
            rtk_combined_pub_ = this->create_publisher<std_msgs::msg::String>("/rtk_raw_data", 10);
            
            // 启动数据接收线程（分离线程避免内存泄漏）
            receive_thread_ = std::thread(&RTKImuNavPublisher::receive_loop, this);
            receive_thread_.detach();
        } else {
            // 串口初始化失败，退出节点
            RCLCPP_FATAL(this->get_logger(), "RTK串口初始化失败！请检查端口权限或设备连接");
            rclcpp::shutdown();
        }
    }

    // 析构函数：关闭串口与线程
    ~RTKImuNavPublisher() {
        running_ = false;
        if (serial_.isOpen()) {
            serial_.close();
        }
    }

private:
    // -------------------------- 成员变量 --------------------------
    // 串口与线程相关
    serial::Serial serial_;          // 串口对象
    std::thread receive_thread_;     // 数据接收线程
    bool running_ = true;            // 线程运行标志
    std::string buf_;                // 数据缓冲区（存储未解析的串口数据）

    // 配置参数
    std::string port_;               // 串口号（如/dev/ttyUSB0）
    int baud_;                       // 波特率
    std::string gps_frame_id_;       // GPS消息的坐标系ID
    std::string imu_frame_id_;       // IMU消息的坐标系ID
    double gyro_scale_;              // 陀螺仪数据转换系数（度→弧度）
    double acc_scale_;               // 加速度计数据转换系数
    int buf_max_size_;               // 缓冲区最大长度（防止内存溢出）

    // ROS发布者
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;       // GPS定位消息发布者
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;             // IMU惯性消息发布者
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rtk_combined_pub_;    // 组合格式消息发布者


    // -------------------------- 成员函数 --------------------------
    // 初始化串口配置
    bool init_serial() {
        try {
            // 设置串口参数
            serial_.setPort(port_);
            serial_.setBaudrate(baud_);
            serial_.setBytesize(serial::eightbits);    // 8位数据位
            serial_.setStopbits(serial::stopbits_one); // 1位停止位
            serial_.setParity(serial::parity_none);    // 无校验位
            serial_.setFlowcontrol(serial::flowcontrol_none); // 无流控

            // 设置超时（读取超时1秒）
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_.setTimeout(timeout);

            // 打开串口
            serial_.open();
            return serial_.isOpen();
        } catch (serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "串口操作异常: %s", e.what());
            return false;
        }
    }

    // 数据接收循环（线程函数）
    void receive_loop() {
        while (running_ && rclcpp::ok()) {
            // 检查串口是否打开且有数据可读
            if (serial_.isOpen() && serial_.available() > 0) {
                // 读取所有可用数据
                std::string raw_data = serial_.read(serial_.available());
                
                // 数据预处理：先删break标记，再删时间戳
                remove_break_tags(raw_data);
                remove_timestamps(raw_data);
                
                // 将预处理后的数据加入缓冲区
                buf_ += raw_data;
                // RCLCPP_DEBUG(this->get_logger(), "接收预处理后的数据: %s", raw_data.c_str());

                // 尝试解析缓冲区中的完整数据包
                parse_complete_packet();

                // 缓冲区溢出保护：超过最大长度时清理
                if (buf_.size() > static_cast<size_t>(buf_max_size_)) {
                    size_t last_start = buf_.find_last_of("CloudTrace5660");
                    if (last_start != std::string::npos) {
                        // 保留最后一个数据包的起始部分，删除前面的无效数据
                        buf_ = buf_.substr(last_start);
                    } else {
                        // 无有效数据包标记，清空缓冲区
                        buf_.clear();
                    }
                    RCLCPP_WARN(this->get_logger(), "数据缓冲区溢出，已清理旧数据");
                }
            }
            // 降低CPU占用（每5ms检查一次）
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    // 解析缓冲区中的完整数据包（格式：CloudTrace5660,POS:...,MAG:...,GYRO:...,ACC:...）
    void parse_complete_packet() {
        // 查找数据包起始标记（CloudTrace5660）
        size_t start_pos = buf_.find("CloudTrace5660");
        if (start_pos == std::string::npos) {
            return; // 无有效数据包，退出
        }

        // 从起始标记开始提取数据（避免解析前面的无效数据）
        std::string data = buf_.substr(start_pos);
        
        // 提取各字段内容（指定结束标记，防止跨字段读取）
        std::string pos_content = extract_field_content(data, "POS:", {"MAG:", "GYRO:", "ACC:"});
        std::string mag_content = extract_field_content(data, "MAG:", {"GYRO:", "ACC:", "CloudTrace5660"});
        std::string gyro_content = extract_field_content(data, "GYRO:", {"MAG:", "ACC:", "CloudTrace5660"});
        std::string acc_content = extract_field_content(data, "ACC:", {"MAG:", "GYRO:", "CloudTrace5660"});

        // 特别处理POS字段：如果POS:后直接是逗号，说明时间戳缺失
        if (pos_content.empty() || (pos_content.size() >= 1 && pos_content[0] == ',')) {
            // RCLCPP_WARN(this->get_logger(), "检测到POS:后直接是逗号，时间戳缺失");
            pos_content = get_current_timestamp() + pos_content; // 在前面插入当前时间戳
        }

        // 验证所有必要字段是否完整（避免解析不完整的数据包）
        if (!pos_content.empty() && !mag_content.empty() && !gyro_content.empty() && !acc_content.empty()) {
            // RCLCPP_DEBUG(this->get_logger(), "提取完整字段: POS=%s, MAG=%s, GYRO=%s, ACC=%s",
            //            pos_content.c_str(), mag_content.c_str(), gyro_content.c_str(), acc_content.c_str());

            // 处理各字段数据（分割+验证+补全）
            std::vector<std::string> pos_parts = split(pos_content, ',');
            std::vector<std::string> processed_pos = process_gps_fields(pos_parts);

            std::vector<std::string> mag_parts = split(mag_content, ',');
            std::vector<std::string> processed_mag = process_imu_fields(mag_parts, "MAG");

            std::vector<std::string> gyro_parts = split(gyro_content, ',');
            std::vector<std::string> processed_gyro = process_imu_fields(gyro_parts, "GYRO");

            std::vector<std::string> acc_parts = split(acc_content, ',');
            std::vector<std::string> processed_acc = process_imu_fields(acc_parts, "ACC");

            // 发布组合格式数据和标准ROS消息
            publish_combined_data(processed_pos, processed_mag, processed_gyro, processed_acc);

            // 移除缓冲区中已解析的数据（避免重复解析）
            size_t end_pos = buf_.find("ACC:" + acc_content) + ("ACC:" + acc_content).length();
            if (end_pos <= buf_.size()) {
                buf_ = buf_.substr(end_pos);
            } else {
                buf_.clear();
            }
        }
    }

    // 发布组合格式数据（符合CloudTrace5660标准格式）
    void publish_combined_data(const std::vector<std::string>& pos, 
                              const std::vector<std::string>& mag,
                              const std::vector<std::string>& gyro,
                              const std::vector<std::string>& acc) {
        std::stringstream ss;
        // 拼接核心标识与POS字段（9位）
        ss << "CloudTrace5660,POS:";
        for (size_t i = 0; i < pos.size(); ++i) {
            if (i > 0) {
                ss << ",";
            }
            ss << pos[i];
        }
        // 拼接MAG字段（3个测量值，已通过process_imu_fields确保有效性）
        ss << ",MAG:";
        for (size_t i = 1; i < mag.size(); ++i) { // 跳过第0位的类型标识（"MAG"）
            if (i > 1) {
                ss << ",";
            }
            ss << mag[i];
        }
        // 拼接GYRO字段（3个测量值）
        ss << ",GYRO:";
        for (size_t i = 1; i < gyro.size(); ++i) { // 跳过第0位的类型标识（"GYRO"）
            if (i > 1) {
                ss << ",";
            }
            ss << gyro[i];
        }
        // 拼接ACC字段（3个测量值）
        ss << ",ACC:";
        for (size_t i = 1; i < acc.size(); ++i) { // 跳过第0位的类型标识（"ACC"）
            if (i > 1) {
                ss << ",";
            }
            ss << acc[i];
        }

        // 发布组合格式消息
        std_msgs::msg::String combined_msg;
        combined_msg.data = ss.str();
        rtk_combined_pub_->publish(combined_msg);
        // 每2秒打印一次发布的消息（避免日志刷屏）
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        //                    "发布组合数据: %s", combined_msg.data.c_str());

        // 同步发布标准ROS消息（供其他节点使用）
        publish_standard_gps(pos);
        publish_standard_imu(mag, gyro, acc);
    }

    // 发布标准ROS NavSatFix消息（GPS定位数据）
    void publish_standard_gps(const std::vector<std::string>& gps_parts) {
        try {
            // 解析GPS核心字段（仅使用必要字段，避免未使用变量警告）
            double latitude = std::stod(gps_parts[1]);  // 纬度
            double longitude = std::stod(gps_parts[2]); // 经度
            double altitude = std::stod(gps_parts[6]);  // 高度
            int fix_quality = std::stoi(gps_parts[3]);  // 定位质量

            // 构造NavSatFix消息
            sensor_msgs::msg::NavSatFix gps_msg;
            gps_msg.header.stamp = this->now();         // 时间戳
            gps_msg.header.frame_id = gps_frame_id_;    // 坐标系ID
            gps_msg.latitude = latitude;                // 纬度（度）
            gps_msg.longitude = longitude;              // 经度（度）
            gps_msg.altitude = altitude;                // 高度（米）

            // 根据定位质量设置状态
            if (fix_quality == 5) {
                gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
            } else if (fix_quality == 4) {
                gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
            } else if (fix_quality == 2) {
                gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
            } else {
                gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
            }
            gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;

            // 设置定位精度（根据HDOP估算，HDOP为gps_parts[5]）
            double hdop = std::stod(gps_parts[5]);
            gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
            double hdop_sq = hdop * hdop;
            gps_msg.position_covariance[0] = hdop_sq;  // 纬度方差
            gps_msg.position_covariance[4] = hdop_sq;  // 经度方差
            gps_msg.position_covariance[8] = 2 * hdop_sq; // 高度方差（放宽精度）

            // 发布GPS消息
            gps_pub_->publish(gps_msg);
        } catch (const std::invalid_argument& e) {
            RCLCPP_ERROR(this->get_logger(), "GPS数据解析失败（无效参数）: %s", e.what());
        } catch (const std::out_of_range& e) {
            RCLCPP_ERROR(this->get_logger(), "GPS数据解析失败（数值超出范围）: %s", e.what());
        }
    }

    // 发布标准ROS Imu消息（惯性测量数据）
    void publish_standard_imu(const std::vector<std::string>& mag,
                             const std::vector<std::string>& gyro,
                             const std::vector<std::string>& acc) {
        try {
            // 解析IMU数据（转换为数值并应用缩放系数）
            // 陀螺仪数据：度→弧度（gyro_scale_ = π/180 ≈ 0.0174533）
            double gyro_x = std::stod(gyro[1]) * gyro_scale_;
            double gyro_y = std::stod(gyro[2]) * gyro_scale_;
            double gyro_z = std::stod(gyro[3]) * gyro_scale_;

            // 加速度计数据（根据硬件输出调整系数，默认g→m/s²）
            double acc_x = std::stod(acc[1]) * acc_scale_;
            double acc_y = std::stod(acc[2]) * acc_scale_;
            double acc_z = std::stod(acc[3]) * acc_scale_;

            // 磁力计数据（直接使用原始值，单位通常为uT）
            double mag_x = std::stod(mag[1]);
            double mag_y = std::stod(mag[2]);
            double mag_z = std::stod(mag[3]);

            // 构造Imu消息
            sensor_msgs::msg::Imu imu_msg;
            imu_msg.header.stamp = this->now();         // 时间戳
            imu_msg.header.frame_id = imu_frame_id_;    // 坐标系ID

            // 角速度（陀螺仪数据）
            imu_msg.angular_velocity.x = gyro_x;
            imu_msg.angular_velocity.y = gyro_y;
            imu_msg.angular_velocity.z = gyro_z;
            imu_msg.angular_velocity_covariance[0] = 0.001; // 角速度方差（示例值，需根据硬件调整）
            imu_msg.angular_velocity_covariance[4] = 0.001;
            imu_msg.angular_velocity_covariance[8] = 0.001;

            // 线性加速度（加速度计数据）
            imu_msg.linear_acceleration.x = acc_x;
            imu_msg.linear_acceleration.y = acc_y;
            imu_msg.linear_acceleration.z = acc_z;
            imu_msg.linear_acceleration_covariance[0] = 0.01; // 加速度方差（示例值）
            imu_msg.linear_acceleration_covariance[4] = 0.01;
            imu_msg.linear_acceleration_covariance[8] = 0.01;

            // 方向（磁力计数据，需结合姿态解算，此处暂存原始值）
            // 注意：标准Imu消息无磁力计字段，若需发布磁力计需自定义消息或使用sensor_msgs/MagneticField
            (void)mag_x; (void)mag_y; (void)mag_z; // 避免未使用变量警告（若需使用可取消注释）

            // 发布Imu消息
            imu_pub_->publish(imu_msg);
        } catch (const std::invalid_argument& e) {
            // RCLCPP_ERROR(this->get_logger(), "IMU数据解析失败（无效参数）: %s", e.what());
        } catch (const std::out_of_range& e) {
            // RCLCPP_ERROR(this->get_logger(), "IMU数据解析失败（数值超出范围）: %s", e.what());
        }
    }
};

// 主函数：初始化ROS节点并运行
int main(int argc, char* argv[]) {
    // 初始化ROS 2
    rclcpp::init(argc, argv);
    // 创建节点实例
    auto node = std::make_shared<RTKImuNavPublisher>();
    // 运行节点（阻塞直到节点关闭）
    rclcpp::spin(node);
    // 关闭ROS 2
    rclcpp::shutdown();
    return 0;
}
