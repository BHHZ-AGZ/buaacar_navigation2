#include <chrono>
#include <memory>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>    
#include <message_filters/sync_policies/approximate_time.h> 
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl/common/io.h>
// #include <pcl/io/io.h>
#include <pcl/point_types.h>

using namespace std::chrono_literals;

class SyncerNode : public rclcpp::Node {
public:
	SyncerNode() : Node("laser_double") {
		RCLCPP_INFO(this->get_logger(), "1111");
		this->declare_parameter("scan1_topic", "/scan1");
		this->get_parameter("scan1_topic", scan1_topic_);
		this->declare_parameter("scan2_topic", "/scan2");
		this->get_parameter("scan2_topic", scan2_topic_);
		this->declare_parameter("scan_topic", "/scan");
		this->get_parameter("scan_topic", scan_topic_);
		this->declare_parameter("lidar1_rotate", 180.0);
		this->get_parameter("lidar1_rotate", lidar1_rotate);
		this->declare_parameter("lidar1_xoffset", -0.275);
		this->get_parameter("lidar1_xoffset", lidar1_xoffset);
		this->declare_parameter("lidar1_yoffset", -0.275);
		this->get_parameter("lidar1_yoffset", lidar1_yoffset);
		this->declare_parameter("lidar2_rotate", 0.0);
		this->get_parameter("lidar2_rotate", lidar2_rotate);
		this->declare_parameter("lidar2_xoffset", 0.275);
		this->get_parameter("lidar2_xoffset", lidar2_xoffset);
		this->declare_parameter("lidar2_yoffset", 0.275);
		this->get_parameter("lidar2_yoffset", lidar2_yoffset);
		rclcpp::QoS qos(10);
		qos.best_effort();
		auto rmw_qos_profile = qos.get_rmw_qos_profile();
		publisher_fusion_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, 10);
		subscriber_scan1_.subscribe(this, scan1_topic_, rmw_qos_profile);
		subscriber_scan2_.subscribe(this, scan2_topic_, rmw_qos_profile);
		laser_sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), subscriber_scan1_, subscriber_scan2_);
		laser_sync_->registerCallback(std::bind(&SyncerNode::LaserSyncCallback, this, std::placeholders::_1, std::placeholders::_2));
	}

private:
	std::string scan_topic_, scan1_topic_, scan2_topic_;
	double lidar1_rotate, lidar2_rotate, lidar1_xoffset, lidar2_xoffset, lidar1_yoffset, lidar2_yoffset;
	float x, y;
	int node_count;

  // This callback is never being called.
	void LaserSyncCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg1, const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg2) {
		node_count = round((msg1->angle_max - msg1->angle_min) / msg1->angle_increment);
		// 创建点云
		pcl::PointCloud<pcl::PointXYZI> pointcloud;
		pcl::PointXYZI newPoint; // 创建一个点
		newPoint.z = 0.0;
		double newPointAngle;
		//将scan1的消息转换为点云
		for(int i = 0; i < node_count; i++)
		{
			if(msg1->ranges[i] == std::numeric_limits<float>::infinity())  //如果没有障碍物则跳过
			{
				continue;
			}
			newPointAngle = msg1->angle_min + msg1->angle_increment * i;

			x = msg1->ranges[i] * cos(newPointAngle);
			y = msg1->ranges[i] * sin(newPointAngle);
			newPoint.x = x * cos(DEG2RAD(lidar1_rotate)) - y * sin(DEG2RAD(lidar1_rotate)) + float(lidar1_xoffset);
			newPoint.y = x * sin(DEG2RAD(lidar1_rotate)) + y * cos(DEG2RAD(lidar1_rotate)) + float(lidar1_yoffset);
			newPoint.intensity = msg1->intensities[i];
			pointcloud.push_back(newPoint);
		}
		//将scan2的消息转换为点云
		node_count = round((msg2->angle_max - msg2->angle_min) / msg2->angle_increment);
		for(int i = 0; i < node_count; i++)
		{   
			//如果没有障碍物则跳过
			if(msg2->ranges[i] == std::numeric_limits<float>::infinity())
			{
				continue;
			}
			newPointAngle = msg2->angle_min + msg2->angle_increment * i;
			
			x = msg2->ranges[i] * cos(newPointAngle);
			y = msg2->ranges[i] * sin(newPointAngle);
			newPoint.x = x * cos(DEG2RAD(lidar2_rotate)) - y * sin(DEG2RAD(lidar2_rotate)) + lidar2_xoffset;
			newPoint.y = x * sin(DEG2RAD(lidar2_rotate)) + y * cos(DEG2RAD(lidar2_rotate)) + lidar2_yoffset;
			newPoint.intensity = msg2->intensities[i];
			pointcloud.push_back(newPoint);
		}

		// 创建LaserScan消息
		sensor_msgs::msg::LaserScan scan_msg;
		scan_msg.header.stamp = this->get_clock()->now();
		scan_msg.header.frame_id = "laser_link";
		scan_msg.angle_min = 0.0;
		scan_msg.angle_max = 2 * M_PI;
		scan_msg.angle_increment = msg1->angle_increment;
		node_count = round((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment);
		// scan_msg.scan_time = (this->get_clock()->now() - scan_msg.header.stamp).toSec();
		// scan_msg.time_increment = scan_msg.scan_time / (double)(node_count);
		scan_msg.range_min = (msg1->range_min) <= (msg2->range_min) ? msg1->range_min : msg2->range_min;
		scan_msg.range_max = (msg1->range_max) >= (msg2->range_max) ? msg1->range_max : msg2->range_max;
		
		//先将数组用inf及0填充
		scan_msg.ranges.assign(node_count, std::numeric_limits<float>::infinity());
		scan_msg.intensities.assign(node_count, 0);

		for(auto point : pointcloud.points)
		{
			float range = hypot(point.x, point.y);
			float angle = atan2(point.y, point.x);
			if (angle < 0.0) angle = 2*M_PI + angle;
			int index = round((angle - scan_msg.angle_min) / scan_msg.angle_increment); // 当前扫描点的索引号
			if (index >= 0 && index < node_count)
			{
				//如果range小于range[index]则赋值
				if (range < scan_msg.ranges[index])
				{
					scan_msg.ranges[index] = range;
					point.intensity = 0 ; //M10 have no intensity
					scan_msg.intensities[index] = point.intensity;
				}
			}
		}
		publisher_fusion_scan_->publish(scan_msg);
	}

	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_fusion_scan_;
	message_filters::Subscriber<sensor_msgs::msg::LaserScan> subscriber_scan1_;
	message_filters::Subscriber<sensor_msgs::msg::LaserScan> subscriber_scan2_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan> SyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> laser_sync_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SyncerNode>());
  rclcpp::shutdown();
  return 0;
}