#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "yaml-cpp/yaml.h"
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include "tf2/LinearMath/Quaternion.h"
#include <chrono>

using namespace std::chrono_literals;

// 从PGM文件读取栅格数据
bool read_pgm(const std::string& filename, std::vector<int8_t>& data, int& width, int& height, int& max_value)
{
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("map_loader"), "无法打开PGM文件: %s", filename.c_str());
        return false;
    }

    std::string magic;
    file >> magic;
    if (magic != "P5") {
        RCLCPP_ERROR(rclcpp::get_logger("map_loader"), "不支持的PGM格式，只支持P5二进制格式");
        return false;
    }

    // 读取宽度、高度和最大值
    file >> width >> height >> max_value;
    file.ignore(); // 忽略换行符

    // 分配数据缓冲区
    size_t data_size = width * height;
    data.resize(data_size);

    // 读取像素数据（灰度值）
    std::vector<uint8_t> pgm_data(data_size);
    file.read(reinterpret_cast<char*>(pgm_data.data()), data_size);

    // 转换为 occupancy grid 格式（0-100表示占据概率，-1表示未知）
    for (size_t i = 0; i < data_size; ++i) {
        // PGM通常是白色为空闲(255)，黑色为障碍物(0)
        // 转换为 occupancy grid 的 conventions: 0为空闲，100为占据
        int value = 100 - static_cast<int>(std::round(static_cast<double>(pgm_data[i]) * 100.0 / max_value));
        if (value < 0) value = 0;
        if (value > 100) value = 100;
        data[i] = static_cast<int8_t>(value);
    }

    RCLCPP_INFO(rclcpp::get_logger("map_loader"), "成功读取PGM文件: %dx%d像素", width, height);
    return true;
}

// 从YAML文件读取地图元数据
bool read_map_yaml(const std::string& filename, std::string& image_path, double& resolution, 
                  double& origin_x, double& origin_y, double& origin_yaw, bool& negate)
{
    try {
        YAML::Node config = YAML::LoadFile(filename);
        
        // 读取必要参数
        image_path = config["image"].as<std::string>();
        resolution = config["resolution"].as<double>();
        
        // 读取原点坐标 (x, y, yaw)
        YAML::Node origin = config["origin"];
        origin_x = origin[0].as<double>();
        origin_y = origin[1].as<double>();
        origin_yaw = origin[2].as<double>();
        
        // 读取可选参数
        negate = config["negate"].as<bool>(false);
        
        RCLCPP_INFO(rclcpp::get_logger("map_loader"), "成功读取YAML文件: 分辨率=%.2fm, 原点=(%.2f, %.2f, %.2f)",
                   resolution, origin_x, origin_y, origin_yaw);
        return true;
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("map_loader"), "YAML解析错误: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("map_loader"), "读取YAML文件错误: %s", e.what());
        return false;
    }
}

class MapLoaderNode : public rclcpp::Node
{
public:
    MapLoaderNode() : Node("map_loader_node")
    {
        // 声明地图文件路径参数
        this->declare_parameter("map_yaml_path", "");
        
        // 获取地图YAML文件路径
        std::string map_yaml_path = this->get_parameter("map_yaml_path").as_string();
        if (map_yaml_path.empty()) {
            RCLCPP_FATAL(this->get_logger(), "请指定地图YAML文件路径: 设置参数 map_yaml_path");
            rclcpp::shutdown();
            return;
        }
        
        // 读取地图元数据
        std::string image_path;
        double resolution, origin_x, origin_y, origin_yaw;
        bool negate;
        
        if (!read_map_yaml(map_yaml_path, image_path, resolution, origin_x, origin_y, origin_yaw, negate)) {
            RCLCPP_FATAL(this->get_logger(), "读取地图YAML文件失败: %s", map_yaml_path.c_str());
            rclcpp::shutdown();
            return;
        }
        
        // 构建完整的图像文件路径（如果是相对路径，则基于YAML文件位置）
        std::string yaml_dir = map_yaml_path.substr(0, map_yaml_path.find_last_of("/\\") + 1);
        std::string full_image_path = yaml_dir + image_path;
        
        // 读取PGM图像数据
        std::vector<int8_t> map_data;
        int width, height, max_value;
        
        if (!read_pgm(full_image_path, map_data, width, height, max_value)) {
            RCLCPP_FATAL(this->get_logger(), "读取地图图像文件失败: %s", full_image_path.c_str());
            rclcpp::shutdown();
            return;
        }
        
        // 如果需要反转占据值
        if (negate) {
            for (auto& val : map_data) {
                if (val != -1) val = 100 - val;
            }
        }
        
        // 创建地图消息
        map_msg_ = std::make_unique<nav_msgs::msg::OccupancyGrid>();
        map_msg_->header.frame_id = "map";
        map_msg_->header.stamp = this->get_clock()->now();
        
        // 设置地图元数据
        map_msg_->info.width = width;
        map_msg_->info.height = height;
        map_msg_->info.resolution = resolution;
        map_msg_->info.origin.position.x = origin_x;
        map_msg_->info.origin.position.y = origin_y;
        map_msg_->info.origin.position.z = 0.0;
        
        // 设置原点姿态（仅yaw有意义）
        tf2::Quaternion q;
        q.setRPY(0, 0, origin_yaw);
        map_msg_->info.origin.orientation.x = q.x();
        map_msg_->info.origin.orientation.y = q.y();
        map_msg_->info.origin.orientation.z = q.z();
        map_msg_->info.origin.orientation.w = q.w();
        
        // 设置地图数据
        map_msg_->data = map_data;
        
        // 创建发布者
        rclcpp::QoS map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", map_qos);
        
        // 发布地图
        map_pub_->publish(*map_msg_);
        RCLCPP_INFO(this->get_logger(), "地图已发布到 /map 话题");
        
        // 创建定时器，定期发布地图（防止地图数据丢失）
        timer_ = this->create_wall_timer(
            5s, // 每5秒发布一次
            std::bind(&MapLoaderNode::publish_map, this)
        );
    }

private:
    // 定期发布地图
    void publish_map()
    {
        if (map_msg_ && map_pub_) {
            map_msg_->header.stamp = this->get_clock()->now();
            map_pub_->publish(*map_msg_);
            RCLCPP_DEBUG(this->get_logger(), "地图已更新发布");
        }
    }
    
    // 地图消息
    std::unique_ptr<nav_msgs::msg::OccupancyGrid> map_msg_;
    
    // 发布者
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapLoaderNode>());
    rclcpp::shutdown();
    return 0;
}
