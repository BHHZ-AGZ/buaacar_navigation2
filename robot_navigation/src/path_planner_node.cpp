#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "robot_navigation/srv/set_pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <vector>
#include <queue>
#include <cmath>
#include <memory>
#include <algorithm>

using namespace std::chrono_literals;

// A*算法节点结构体（避免与rclcpp::Node冲突）
struct AStarNode {
    int x;          // 栅格x坐标
    int y;          // 栅格y坐标
    double g_cost;  // 从起点到当前节点的代价
    double h_cost;  // 启发式代价
    double f_cost;  // 总代价 (f = g + h)
    AStarNode* parent;   // 父节点指针

    AStarNode(int x_, int y_) : x(x_), y(y_), g_cost(0), h_cost(0), f_cost(0), parent(nullptr) {}
};

// 优先队列排序规则
struct CompareNode {
    bool operator()(AStarNode* a, AStarNode* b) {
        return a->f_cost > b->f_cost;  // 小的f_cost优先
    }
};

class DiffDrivePathPlannerNode : public rclcpp::Node
{
public:
    DiffDrivePathPlannerNode() : Node("diff_drive_path_planner_node")
    {
        // 声明矩形底盘尺寸参数（单位：米）
        this->declare_parameter("base_length", 0.5);  // 底盘长度（前后方向）
        this->declare_parameter("base_width", 0.3);   // 底盘宽度（左右方向）
        this->get_parameter("base_length", base_length_);
        this->get_parameter("base_width", base_width_);

        // 订阅地图话题
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10, std::bind(&DiffDrivePathPlannerNode::map_callback, this, std::placeholders::_1));
        
        // 发布路径话题
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);
        
        // 创建设置目标点的服务
        set_goal_srv_ = this->create_service<robot_navigation::srv::SetPose>(
            "set_nav_goal",
            std::bind(&DiffDrivePathPlannerNode::set_goal_callback, this,
                      std::placeholders::_1, std::placeholders::_2));
        
        // 创建TF缓冲区和监听器
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // 创建定时器（差速底盘规划频率可适当降低）
        timer_ = this->create_wall_timer(
            200ms, std::bind(&DiffDrivePathPlannerNode::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Diff-Drive Path Planner Node initialized");
        RCLCPP_INFO(this->get_logger(), "Rectangular base size: length=%.2fm, width=%.2fm",
                   base_length_, base_width_);
    }

private:
    // 地图回调：保存地图并计算底盘栅格尺寸
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_ = *msg;
        map_available_ = true;

        // 将底盘尺寸从米转换为栅格数（向上取整，确保安全）
        base_length_grid_ = static_cast<int>(std::ceil(base_length_ / map_.info.resolution));
        base_width_grid_ = static_cast<int>(std::ceil(base_width_ / map_.info.resolution));
        // 底盘半长/半宽（栅格），用于碰撞检测
        half_length_grid_ = base_length_grid_ / 2;
        half_width_grid_ = base_width_grid_ / 2;

        RCLCPP_INFO(this->get_logger(), "Map received, base size in grid: length=%d, width=%d",
                   base_length_grid_, base_width_grid_);
    }

    // 设置目标点服务回调
    void set_goal_callback(
        const std::shared_ptr<robot_navigation::srv::SetPose::Request> request,
        std::shared_ptr<robot_navigation::srv::SetPose::Response> response)
    {
        if (!map_available_) {
            response->success = false;
            response->message = "Map not available";
            return;
        }
        
        int goal_x, goal_y;
        if (!world_to_grid(request->pose.position.x, request->pose.position.y, goal_x, goal_y) ||
            is_rectangle_collision(goal_x, goal_y, 0)) {  // 检查目标点是否碰撞（朝向0弧度）
            response->success = false;
            response->message = "Invalid goal (collision or out of bounds)";
            return;
        }
        
        goal_pose_ = request->pose;
        goal_set_ = true;
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "Goal set to: (%.2f, %.2f)",
                   goal_pose_.position.x, goal_pose_.position.y);
    }

    // 定时器回调：规划路径
    void timer_callback()
    {
        if (!map_available_ || !goal_set_) return;
        
        // 获取机器人当前位置和朝向
        geometry_msgs::msg::PoseStamped current_pose;
        if (!get_robot_pose(current_pose)) return;
        
        // 转换为栅格坐标
        int start_x, start_y, goal_x, goal_y;
        if (!world_to_grid(current_pose.pose.position.x, current_pose.pose.position.y, start_x, start_y) ||
            !world_to_grid(goal_pose_.position.x, goal_pose_.position.y, goal_x, goal_y)) {
            return;
        }
        
        // 检查起点碰撞（使用当前朝向）
        double current_yaw = get_yaw_from_quaternion(current_pose.pose.orientation);
        if (is_rectangle_collision(start_x, start_y, current_yaw)) {
            RCLCPP_WARN(this->get_logger(), "Robot is in collision");
            return;
        }
        
        // 执行A*搜索（带矩形碰撞检测）
        std::vector<AStarNode*> path_nodes = a_star_search(start_x, start_y, goal_x, goal_y);
        
        if (path_nodes.empty()) {
            RCLCPP_WARN(this->get_logger(), "No path found");
            return;
        }
        
        // 转换为ROS Path消息（优化差速底盘朝向）
        nav_msgs::msg::Path path_msg = convert_nodes_to_path(path_nodes);
        path_pub_->publish(path_msg);
        
        // 释放内存
        for (AStarNode* node : path_nodes) delete node;
    }

    // A*路径搜索（核心算法）
    std::vector<AStarNode*> a_star_search(int start_x, int start_y, int goal_x, int goal_y)
    {
        std::vector<AStarNode*> path;
        if (start_x == goal_x && start_y == goal_y) return path;
        
        // 开放列表和关闭列表
        std::priority_queue<AStarNode*, std::vector<AStarNode*>, CompareNode> open_list;
        std::vector<std::vector<bool>> closed_list(
            map_.info.height, std::vector<bool>(map_.info.width, false));
        std::vector<std::vector<double>> g_cost_map(
            map_.info.height, std::vector<double>(map_.info.width, INFINITY));
        
        // 起点节点
        AStarNode* start_node = new AStarNode(start_x, start_y);
        start_node->g_cost = 0;
        start_node->h_cost = calculate_heuristic(start_x, start_y, goal_x, goal_y);
        start_node->f_cost = start_node->g_cost + start_node->h_cost;
        open_list.push(start_node);
        g_cost_map[start_y][start_x] = 0;
        
        // 差速底盘适合4方向移动（减少对角线移动，避免原地旋转）
        int dirs[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};  // 仅前后左右
        
        while (!open_list.empty()) {
            AStarNode* current_node = open_list.top();
            open_list.pop();
            
            // 到达目标
            if (current_node->x == goal_x && current_node->y == goal_y) {
                AStarNode* temp = current_node;
                while (temp != nullptr) {
                    path.push_back(temp);
                    temp = temp->parent;
                }
                std::reverse(path.begin(), path.end());
                return path;
            }
            
            closed_list[current_node->y][current_node->x] = true;
            
            // 探索邻居
            for (int i = 0; i < 4; i++) {  // 仅4方向移动
                int new_x = current_node->x + dirs[i][0];
                int new_y = current_node->y + dirs[i][1];
                
                // 检查边界和碰撞
                if (new_x < 0 || new_x >= static_cast<int>(map_.info.width) ||
                    new_y < 0 || new_y >= static_cast<int>(map_.info.height)) {
                    continue;
                }
                
                // 计算当前朝向（用于矩形碰撞检测）
                double yaw = 0;
                if (current_node->parent != nullptr) {
                    yaw = std::atan2(
                        new_y - current_node->y, 
                        new_x - current_node->x
                    );
                }
                
                // 检查矩形底盘是否碰撞
                if (is_rectangle_collision(new_x, new_y, yaw) || closed_list[new_y][new_x]) {
                    continue;
                }
                
                // 更新代价
                double new_g_cost = current_node->g_cost + 1.0;  // 4方向移动代价固定为1
                if (new_g_cost < g_cost_map[new_y][new_x]) {
                    AStarNode* neighbor_node = new AStarNode(new_x, new_y);
                    neighbor_node->parent = current_node;
                    neighbor_node->g_cost = new_g_cost;
                    neighbor_node->h_cost = calculate_heuristic(new_x, new_y, goal_x, goal_y);
                    neighbor_node->f_cost = neighbor_node->g_cost + neighbor_node->h_cost;
                    
                    g_cost_map[new_y][new_x] = new_g_cost;
                    open_list.push(neighbor_node);
                }
            }
        }
        
        return path;  // 无路径
    }

    // 矩形底盘碰撞检测（核心修改）
    bool is_rectangle_collision(int center_x, int center_y, double yaw)
    {
        // 生成矩形底盘的4个角点（栅格坐标）
        std::vector<std::pair<int, int>> corners;
        
        // 底盘半长和半宽（栅格）
        double l = half_length_grid_ * map_.info.resolution;
        double w = half_width_grid_ * map_.info.resolution;
        
        // 四个角落的相对坐标（米）
        std::vector<std::pair<double, double>> rel_corners = {
            {l, w},   // 右前
            {l, -w},  // 左前
            {-l, -w}, // 左后
            {-l, w}   // 右后
        };
        
        // 旋转并转换为世界坐标
        for (auto& rc : rel_corners) {
            // 旋转公式：绕中心旋转yaw角
            double x = rc.first * cos(yaw) - rc.second * sin(yaw);
            double y = rc.first * sin(yaw) + rc.second * cos(yaw);
            
            // 转换为世界坐标
            double world_x, world_y;
            grid_to_world(center_x, center_y, world_x, world_y);
            world_x += x;
            world_y += y;
            
            // 转换回栅格坐标并检查是否碰撞
            int grid_x, grid_y;
            if (!world_to_grid(world_x, world_y, grid_x, grid_y)) {
                return true;  // 超出地图范围视为碰撞
            }
            
            // 检查该栅格是否为障碍物
            int index = grid_y * map_.info.width + grid_x;
            if (map_.data[index] > 50 || map_.data[index] == -1) {
                return true;  // 碰撞
            }
        }
        
        return false;  // 无碰撞
    }

    // 计算启发式代价（欧几里得距离）
    double calculate_heuristic(int x1, int y1, int x2, int y2)
    {
        double dx = x1 - x2;
        double dy = y1 - y2;
        return std::sqrt(dx*dx + dy*dy);
    }

    // 坐标转换：世界坐标 -> 栅格坐标
    bool world_to_grid(double world_x, double world_y, int& grid_x, int& grid_y)
    {
        grid_x = static_cast<int>((world_x - map_.info.origin.position.x) / map_.info.resolution);
        grid_y = static_cast<int>((world_y - map_.info.origin.position.y) / map_.info.resolution);
        
        if (grid_x < 0 || grid_x >= static_cast<int>(map_.info.width) ||
            grid_y < 0 || grid_y >= static_cast<int>(map_.info.height)) {
            return false;
        }
        return true;
    }

    // 坐标转换：栅格坐标 -> 世界坐标
    void grid_to_world(int grid_x, int grid_y, double& world_x, double& world_y)
    {
        world_x = map_.info.origin.position.x + (grid_x + 0.5) * map_.info.resolution;
        world_y = map_.info.origin.position.y + (grid_y + 0.5) * map_.info.resolution;
    }

    // 转换路径节点为ROS Path消息（优化朝向）
    nav_msgs::msg::Path convert_nodes_to_path(const std::vector<AStarNode*>& nodes)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = map_.header.frame_id;
        path_msg.header.stamp = this->get_clock()->now();
        
        for (size_t i = 0; i < nodes.size(); i++) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            
            // 位置
            grid_to_world(nodes[i]->x, nodes[i]->y, pose.pose.position.x, pose.pose.position.y);
            pose.pose.position.z = 0.0;
            
            // 朝向（差速底盘只能沿运动方向前进）
            if (i < nodes.size() - 1) {
                // 朝向 next node
                double dx = nodes[i+1]->x - nodes[i]->x;
                double dy = nodes[i+1]->y - nodes[i]->y;
                double yaw = std::atan2(dy, dx);
                
                // 转换为四元数
                tf2::Quaternion quat;
                quat.setRPY(0, 0, yaw);
                pose.pose.orientation = tf2::toMsg(quat);
            } else {
                // 最后一个点使用目标朝向
                pose.pose.orientation = goal_pose_.orientation;
            }
            
            path_msg.poses.push_back(pose);
        }
        
        return path_msg;
    }

    // 获取机器人当前位姿（从TF）
    bool get_robot_pose(geometry_msgs::msg::PoseStamped& pose)
    {
        try {
            auto transform = tf_buffer_->lookupTransform(
                map_.header.frame_id, "base_link", tf2::TimePointZero);
            
            pose.header.frame_id = map_.header.frame_id;
            pose.header.stamp = this->get_clock()->now();
            pose.pose.position.x = transform.transform.translation.x;
            pose.pose.position.y = transform.transform.translation.y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation = transform.transform.rotation;
            return true;
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "TF error: %s", ex.what());
            return false;
        }
    }

    // 从四元数获取偏航角（yaw）
    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& quat)
    {
        tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    // 成员变量
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Service<robot_navigation::srv::SetPose>::SharedPtr set_goal_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    nav_msgs::msg::OccupancyGrid map_;
    bool map_available_ = false;
    
    geometry_msgs::msg::Pose goal_pose_;
    bool goal_set_ = false;
    
    // 矩形底盘参数（米）
    double base_length_;  // 长度（前后方向）
    double base_width_;   // 宽度（左右方向）
    
    // 矩形底盘参数（栅格）
    int base_length_grid_;
    int base_width_grid_;
    int half_length_grid_;  // 半长（栅格）
    int half_width_grid_;   // 半宽（栅格）
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffDrivePathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
