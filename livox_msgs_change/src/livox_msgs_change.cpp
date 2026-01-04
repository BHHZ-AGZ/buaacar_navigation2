/**
 * @file livox_msgs_change.cpp
 * @brief Livox CustomMsg 转 PointCloud2（修复Field xyzi does not exist错误）
 */
#include <rclcpp/rclcpp.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class LivoxCustomMsgToPointCloud2 : public rclcpp::Node
{
public:
    LivoxCustomMsgToPointCloud2() : Node("livox_custommsg_to_pointcloud2")
    {
        // 配置参数
        this->declare_parameter("sub_topic", "/livox/lidar");
        this->declare_parameter("pub_topic", "/livox/points");
        this->declare_parameter("qos_depth", 10);

        // 获取参数值
        std::string sub_topic = this->get_parameter("sub_topic").as_string();
        std::string pub_topic = this->get_parameter("pub_topic").as_string();
        int qos_depth = this->get_parameter("qos_depth").as_int();

        // 创建订阅器（CustomMsg 自定义消息）
        custom_msg_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            sub_topic,
            rclcpp::QoS(qos_depth),
            std::bind(&LivoxCustomMsgToPointCloud2::customMsgCallback, this, std::placeholders::_1)
        );

        // 创建发布器（PointCloud2 标准消息）
        pointcloud2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            pub_topic,
            rclcpp::QoS(qos_depth)
        );

        // 日志提示
        RCLCPP_INFO(this->get_logger(), "节点启动成功！订阅: %s | 发布: %s", sub_topic.c_str(), pub_topic.c_str());
    }

private:
    void customMsgCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr custom_msg)
    {
        // 1. 初始化 PointCloud2 消息
        auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        cloud_msg->header = custom_msg->header;
        cloud_msg->height = 1;                  // 无序点云
        cloud_msg->width = custom_msg->point_num;// 点数量
        cloud_msg->is_dense = false;            // 允许空点
        cloud_msg->is_bigendian = false;        // 小端序

        // 2. 显式定义 PointCloud2 字段（核心修复：替代xyzi快捷字符串）
        // 字段说明：x(0字节)、y(4字节)、z(8字节)、intensity(12字节)，均为float32
        std::vector<sensor_msgs::msg::PointField> fields;
        // X字段
        sensor_msgs::msg::PointField x_field;
        x_field.name = "x";
        x_field.offset = 0;
        x_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        x_field.count = 1;
        // Y字段
        sensor_msgs::msg::PointField y_field;
        y_field.name = "y";
        y_field.offset = 4;
        y_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        y_field.count = 1;
        // Z字段
        sensor_msgs::msg::PointField z_field;
        z_field.name = "z";
        z_field.offset = 8;
        z_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        z_field.count = 1;
        // 强度字段（intensity）
        sensor_msgs::msg::PointField i_field;
        i_field.name = "intensity";  // 注意：字段名是intensity，不是i
        i_field.offset = 12;
        i_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        i_field.count = 1;

        // 将字段添加到PointCloud2
        fields = {x_field, y_field, z_field, i_field};
        cloud_msg->fields = fields;
        cloud_msg->point_step = 16;  // 单个点总字节数：4*4=16（x+y+z+intensity）
        cloud_msg->row_step = cloud_msg->point_step * cloud_msg->width;  // 整行字节数
        cloud_msg->data.resize(cloud_msg->row_step);  // 分配内存

        // 3. 填充点云数据
        // 初始化迭代器（注意：intensity字段名要和上面定义的一致）
        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_i(*cloud_msg, "intensity");

        // 遍历所有点（i改为unsigned int，匹配point_num的类型）
        for (unsigned int i = 0; i < custom_msg->point_num; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_i)
        {
            const livox_ros_driver2::msg::CustomPoint& livox_point = custom_msg->points[i];
            *iter_x = livox_point.x;                // X坐标
            *iter_y = livox_point.y;                // Y坐标
            *iter_z = livox_point.z;                // Z坐标
            *iter_i = static_cast<float>(livox_point.reflectivity);  // 反射强度
        }

        // 4. 发布转换后的PointCloud2
        pointcloud2_pub_->publish(*cloud_msg);
        RCLCPP_DEBUG(this->get_logger(), "成功发布PointCloud2，点数：%u", custom_msg->point_num);
    }

    // 订阅器/发布器
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_msg_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LivoxCustomMsgToPointCloud2>());
    rclcpp::shutdown();
    return 0;
}