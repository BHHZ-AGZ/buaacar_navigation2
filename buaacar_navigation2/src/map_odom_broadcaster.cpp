#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

class MapOdomBroadcaster : public rclcpp::Node
{
public:
    MapOdomBroadcaster() : Node("map_odom_broadcaster")
    {
        // 初始化TF广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // 声明参数（修正函数名拼写错误）
        this->declare_parameter<double>("x", -0.648);
        this->declare_parameter<double>("y", 0.224);
        this->declare_parameter<double>("z", 0.0);
        this->declare_parameter<double>("roll", 0.0);
        this->declare_parameter<double>("pitch", 0.0);
        this->declare_parameter<double>("yaw", 0.785);
        this->declare_parameter<double>("publish_rate", 10.0); // 修正函数名
        //      x: -0.6477585434913635
        //      y: 0.22448429465293884

        // 获取发布频率参数
        double publish_rate;
        this->get_parameter("publish_rate", publish_rate);
        
        // 创建定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(1000.0 / publish_rate)),
            std::bind(&MapOdomBroadcaster::broadcast_transform, this)
        );

        RCLCPP_INFO(this->get_logger(), "Map to Odom TF broadcaster initialized");
        RCLCPP_INFO(this->get_logger(), "Publish rate: %.2f Hz", publish_rate);
    }

private:
    void broadcast_transform()
    {
        // 创建变换消息
        geometry_msgs::msg::TransformStamped transform;
        
        // 设置变换的时间戳和坐标系
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "odom";
        
        // 从参数获取平移值（修正消息结构访问错误）
        transform.transform.translation.x = this->get_parameter("x").as_double();
        transform.transform.translation.y = this->get_parameter("y").as_double();
        transform.transform.translation.z = this->get_parameter("z").as_double();
        
        // 从参数获取旋转值并转换为四元数
        tf2::Quaternion q;
        q.setRPY(
            this->get_parameter("roll").as_double(),
            this->get_parameter("pitch").as_double(),
            this->get_parameter("yaw").as_double()
        );
        
        // 设置旋转四元数（修正消息结构访问错误）
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        
        // 发布变换
        tf_broadcaster_->sendTransform(transform);
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapOdomBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
