#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

class PointCloudToLaserScan : public rclcpp::Node
{
public:
  PointCloudToLaserScan()
  : Node("pointcloud_to_laserscan"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // 参数配置
    this->declare_parameter("min_height", -0.1);
    this->declare_parameter("max_height", 0.3);
    this->declare_parameter("angle_min", -M_PI);
    this->declare_parameter("angle_max", M_PI);
    this->declare_parameter("angle_increment", M_PI / 180.0);
    this->declare_parameter("scan_time", 0.1);
    this->declare_parameter("range_min", 0.1);
    this->declare_parameter("range_max", 5.0);
    this->declare_parameter("outlier_mean_k", 10); //10
    this->declare_parameter("outlier_stddev", 10.0);    //1
    this->declare_parameter("target_frame", "base_link");
    this->declare_parameter("z_projection_min", 9.9);  // 新增：Z轴投影的最小高度
    this->declare_parameter("z_projection_max", 10.0);  // 新增：Z轴投影的最大高度

    // 创建订阅者和发布者 /livox/lidar  /unilidar/cloud
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", 10,
      std::bind(&PointCloudToLaserScan::cloudCallback, this, std::placeholders::_1));
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    try {
      // 获取参数值
      double min_height = this->get_parameter("min_height").as_double();
      double max_height = this->get_parameter("max_height").as_double();
      double range_min = this->get_parameter("range_min").as_double();
      double range_max = this->get_parameter("range_max").as_double();
      int outlier_mean_k = this->get_parameter("outlier_mean_k").as_int();
      double outlier_stddev = this->get_parameter("outlier_stddev").as_double();
      std::string target_frame = this->get_parameter("target_frame").as_string();
      double z_projection_min = this->get_parameter("z_projection_min").as_double();  // 新增
      double z_projection_max = this->get_parameter("z_projection_max").as_double();  // 新增

      // 转换点云到目标坐标系 (base_link)
      auto transformed_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
      tf2::doTransform(*cloud_msg, *transformed_cloud,
                      tf_buffer_.lookupTransform(
                        target_frame, cloud_msg->header.frame_id,
                        cloud_msg->header.stamp, rclcpp::Duration::from_seconds(1.0)));

      // 转换为PCL点云
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(*transformed_cloud, *pcl_cloud);

      // 高度过滤
      pcl::PointCloud<pcl::PointXYZ>::Ptr height_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      for (const auto& point : *pcl_cloud) {
        if (point.z >= min_height && point.z <= max_height) {
          height_filtered_cloud->points.push_back(point);
        }
      }

      // 离群点过滤
      pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setInputCloud(height_filtered_cloud);
      sor.setMeanK(outlier_mean_k);
      sor.setStddevMulThresh(outlier_stddev);
      sor.filter(*filtered_cloud);

      // 创建LaserScan消息
      auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
      scan_msg->header = transformed_cloud->header;
      scan_msg->header.frame_id = target_frame;
      scan_msg->angle_min = this->get_parameter("angle_min").as_double();
      scan_msg->angle_max = this->get_parameter("angle_max").as_double();
      scan_msg->angle_increment = this->get_parameter("angle_increment").as_double();
      scan_msg->time_increment = 0.0;
      scan_msg->scan_time = this->get_parameter("scan_time").as_double();
      scan_msg->range_min = range_min;
      scan_msg->range_max = range_max;

      // 计算扫描点数
      uint32_t ranges_size = std::ceil(
        (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
      scan_msg->ranges.assign(ranges_size, std::numeric_limits<float>::infinity());

      // 填充距离数据 - 确保使用base_link坐标系的XY平面
      for (const auto& point : filtered_cloud->points) {
        // 检查点的Z坐标是否在投影范围内
        if (point.z < z_projection_min || point.z > z_projection_max) {
          continue;
        }
        
        // 计算XY平面上的距离 (忽略Z坐标)
        float range = std::hypot(point.x, point.y);
        if (range < range_min || range > range_max) continue;
        
        // 计算XY平面上的角度 (忽略Z坐标)
        float angle = std::atan2(point.y, point.x);
        if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) continue;
        
        // 计算对应的索引
        int index = static_cast<int>((angle - scan_msg->angle_min) / scan_msg->angle_increment);
        if (index >= 0 && index < static_cast<int>(ranges_size)) {
          // 只保留最近的点
          if (range < scan_msg->ranges[index]) {
            scan_msg->ranges[index] = range;
          }
        }
      }

      // 发布扫描数据
      scan_pub_->publish(std::move(scan_msg));

    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "TF转换失败: %s", ex.what());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "处理点云时出错: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudToLaserScan>());
  rclcpp::shutdown();
  return 0;
}