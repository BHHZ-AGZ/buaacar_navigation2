// navigation_goal.cpp
#include "navigation_goal.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include "action_msgs/msg/goal_status.hpp"
#include <boost/beast/http.hpp>

// 构造函数：初始化发布者、订阅者
NavigationGoalHandler::NavigationGoalHandler(
    rclcpp::Node::SharedPtr node,
    std::unique_ptr<BaseLinkPoseCalculator>& pose_calculator
) : node_(node), pose_calculator_(pose_calculator) { 
    // 初始化导航目标发布者（兼容Nav2）
    goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 
        10  // QoS配置
    );
    // 初始化全局路径订阅者
    path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
        "/plan", 
        10, 
        std::bind(&NavigationGoalHandler::path_callback, this, std::placeholders::_1)
    );
    path_planner_client_ = rclcpp_action::create_client<nav2_msgs::action::ComputePathToPose>(
        node_,  // 节点指针
        "/compute_path_to_pose"  // 动作话题（Nav2默认路径规划动作）
    );
    navigate_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        node_,
        "/navigate_to_pose"  // 导航执行动作话题（Nav2默认）
    );
    ultrasound_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/ultrasound_data", 10, std::bind(&NavigationGoalHandler::ultrasound_callback, this, std::placeholders::_1));
    init_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    RCLCPP_INFO(node_->get_logger(), "NavigationGoalHandler初始化完成");
}

// 全局路径回调：更新最新路径缓存
void NavigationGoalHandler::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(path_mutex_);
    latest_path_ = *msg;  // 缓存最新路径
    RCLCPP_DEBUG(node_->get_logger(), "收到全局路径，包含%zu个点", msg->poses.size());
}

// 栅格X转物理X
double NavigationGoalHandler::grid_to_physical_x(int grid_x, const MapParameters& map_params) {
    return map_params.origin_x + grid_x * map_params.resolution;
}

// 栅格Y转物理Y
double NavigationGoalHandler::grid_to_physical_y(int grid_y, const MapParameters& map_params) {
    return map_params.origin_y + (map_params.height - 1 - grid_y) * map_params.resolution;
}

// 物理X转栅格X（四舍五入）
int NavigationGoalHandler::physical_to_grid_x(double x, const MapParameters& map_params) {
    if (map_params.resolution <= 0) return 0;
    return static_cast<int>((x - map_params.origin_x) / map_params.resolution + 0.5);
}

// 物理Y转栅格Y（四舍五入）
int NavigationGoalHandler::physical_to_grid_y(double y, const MapParameters& map_params) {
    if (map_params.resolution <= 0) return 0;
    double raw_grid_y = (y - map_params.origin_y) / map_params.resolution;
    return static_cast<int>(map_params.height - 1 - raw_grid_y + 0.5); 
}

// 核心处理函数：解析请求→发布目标→返回路径
NavigationResult NavigationGoalHandler::handle_goal(const nlohmann::json& req_data) {
    NavigationResult result;
    result.successed = false;
    result.errCode = 0;
    result.msg = "";

    try {
        // 1. 验证请求参数
        if (!req_data.contains("target_grid_x") || 
            !req_data.contains("target_grid_y") || 
            !req_data.contains("target_angle")) {
            result.errCode = 4;
            result.msg = "缺少必要字段（target_grid_x、target_grid_y、target_angle）";
            return result;
        }

        // 2. 提取栅格坐标和角度
        const int target_grid_x = req_data["target_grid_x"].get<int>();
        const int target_grid_y = req_data["target_grid_y"].get<int>();
        const double target_angle = req_data["target_angle"].get<double>();  // 度

        // 3. 获取地图参数（用于坐标转换）
        const MapParameters map_params = pose_calculator_->get_map_parameters();
        if (map_params.resolution <= 0) {
            result.errCode = 7;
            result.msg = "地图分辨率无效（<=0），无法转换坐标";
            return result;
        }

        // 4. 栅格坐标→物理坐标（米）
        const double target_x = grid_to_physical_x(target_grid_x, map_params);
        const double target_y = grid_to_physical_y(target_grid_y, map_params);
        RCLCPP_INFO(node_->get_logger(), 
                   "导航目标转换完成: 栅格(%d,%d)→物理(%.2f,%.2f)，角度%.2f°",
                   target_grid_x, target_grid_y, target_x, target_y, target_angle);

        // 5. 发布导航目标消息（PoseStamped）
        geometry_msgs::msg::PoseStamped goal_msg;
        goal_msg.header.frame_id = "map";  // 地图坐标系
        goal_msg.header.stamp = node_->now();
        goal_msg.pose.position.x = target_x;
        goal_msg.pose.position.y = target_y;
        goal_msg.pose.position.z = 0.0;  // 2D导航

        // 角度转四元数（度→弧度）
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, target_angle * M_PI / 180.0);  // 仅偏航角
        goal_msg.pose.orientation.x = quat.x();
        goal_msg.pose.orientation.y = quat.y();
        goal_msg.pose.orientation.z = quat.z();
        goal_msg.pose.orientation.w = quat.w();
        goal_pub_->publish(goal_msg);

        // 6. 转换全局路径为栅格坐标
        std::lock_guard<std::mutex> lock(path_mutex_);
        if (latest_path_.poses.empty()) {
            result.errCode = 8;
            result.msg = "未获取到全局路径（可能导航未规划或规划失败）";
            return result;
        }

        // 遍历路径点，转换为栅格坐标
        for (const auto& pose_stamped : latest_path_.poses) {
            const auto& pos = pose_stamped.pose.position;
            const int grid_x = physical_to_grid_x(pos.x, map_params);
            const int grid_y = physical_to_grid_y(pos.y, map_params);
            result.grid_phits.emplace_back(grid_x, grid_y);
        }

        // 7. 处理成功
        result.successed = true;
        result.errCode = 0;
        result.msg = "数据获取成功";
        return result;

    } catch (const nlohmann::json::exception& e) {
        // JSON解析错误
        result.errCode = 3;
        result.msg = "JSON解析失败: " + std::string(e.what());
    } catch (const std::exception& e) {
        // 其他异常
        result.errCode = 5;
        result.msg = "处理失败: " + std::string(e.what());
    }

    return result;
}

NavigationResult NavigationGoalHandler::plan_path(const nlohmann::json& req_data) {
    NavigationResult result;
    result.successed = false;
    result.errCode = 0;
    result.msg = "";

    try {
        // 1. 验证请求参数（保持不变）
        if (!req_data.contains("start") || !req_data["start"].contains("x") || !req_data["start"].contains("y") ||
            !req_data.contains("end") || !req_data["end"].contains("x") || !req_data["end"].contains("y")) {
            result.errCode = 4;
            result.msg = "缺少必要字段（start.x, start.y, end.x, end.y）";
            return result;
        }

        // 2. 提取起始和目标栅格坐标（保持不变）
        const int start_grid_x = req_data["start"]["x"].get<int>();
        const int start_grid_y = req_data["start"]["y"].get<int>();
        const int end_grid_x = req_data["end"]["x"].get<int>();
        const int end_grid_y = req_data["end"]["y"].get<int>();
        RCLCPP_INFO(node_->get_logger(), "路径规划请求：起始(%d,%d) → 目标(%d,%d)",
                   start_grid_x, start_grid_y, end_grid_x, end_grid_y);

        // 3. 栅格坐标转物理坐标（保持不变）
        const MapParameters map_params = pose_calculator_->get_map_parameters();
        if (map_params.resolution <= 0) {
            result.errCode = 7;
            result.msg = "地图分辨率无效，无法转换坐标";
            return result;
        }
        const double start_x = grid_to_physical_x(start_grid_x, map_params);
        const double start_y = grid_to_physical_y(start_grid_y, map_params);
        const double goal_x = grid_to_physical_x(end_grid_x, map_params);
        const double goal_y = grid_to_physical_y(end_grid_y, map_params);

        // 4. 构造动作目标消息（修正：使用动作的Goal消息）
        auto goal_msg = nav2_msgs::action::ComputePathToPose::Goal();
        goal_msg.start.header.frame_id = "map";
        goal_msg.start.pose.position.x = start_x;
        goal_msg.start.pose.position.y = start_y;
        goal_msg.start.pose.orientation.w = 1.0;
        goal_msg.goal.header.frame_id = "map";
        goal_msg.goal.pose.position.x = goal_x;
        goal_msg.goal.pose.position.y = goal_y;
        goal_msg.goal.pose.orientation.w = 1.0;
        goal_msg.planner_id = "";
        goal_msg.use_start = true;

        // 5. 发送动作目标并等待结果
        if (!path_planner_client_->action_server_is_ready()) {
            result.errCode = 9;
            result.msg = "路径规划动作服务器未就绪";
            return result;
        }

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SendGoalOptions();
        auto future = path_planner_client_->async_send_goal(goal_msg, send_goal_options);

        if (future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
            result.errCode = 9;
            result.msg = "路径规划超时（10秒未响应）";
            return result;
        }

        auto goal_handle = future.get();
        if (!goal_handle) {
            result.errCode = 10;
            result.msg = "发送路径规划目标失败";
            return result;
        }

        auto result_future = path_planner_client_->async_get_result(goal_handle);
        if (result_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
            result.errCode = 9;
            result.msg = "获取路径规划结果超时";
            return result;
        }

        // 6. 解析动作结果（兼容版本修改）
        auto action_result = result_future.get();
        if (action_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            result.errCode = 10;
            std::string code_str;
            switch (action_result.code) {
                case rclcpp_action::ResultCode::ABORTED: 
                    code_str = "规划器无法生成路径（可能坐标在障碍物或无连通路径）"; 
                    break;
                case rclcpp_action::ResultCode::CANCELED: 
                    code_str = "规划被取消"; 
                    break;
                default: 
                    code_str = "结果码: " + std::to_string(static_cast<int>(action_result.code)); 
                    break;
            }
            result.msg = "路径规划失败: " + code_str;
            return result;
        }

        // 7. 检查路径是否有效
        const auto& path = action_result.result->path;
        if (path.poses.empty()) {
            result.errCode = 10;
            result.msg = "路径规划失败，无有效路径";
            return result;
        }

        // 8. 转换路径点为栅格坐标（保持不变）
        for (const auto& pose_stamped : path.poses) {
            const int grid_x = physical_to_grid_x(pose_stamped.pose.position.x, map_params);
            const int grid_y = physical_to_grid_y(pose_stamped.pose.position.y, map_params);
            result.grid_phits.emplace_back(grid_x, grid_y);
        }

        // 处理成功
        result.successed = true;
        result.errCode = 0;
        result.msg = "数据获取成功";
        return result;

    } catch (const nlohmann::json::exception& e) {
        result.errCode = 3;
        result.msg = "JSON解析失败: " + std::string(e.what());
    } catch (const std::exception& e) {
        result.errCode = 5;
        result.msg = "处理失败: " + std::string(e.what());
    }

    return result;
}
NavigationResult NavigationGoalHandler::handle_multi_goals(const nlohmann::json& req_data) {
    NavigationResult result;
    result.successed = false;
    result.errCode = 0;
    result.msg = "";
    result.data = "";
    try {
        // 1. 打印原始请求数据（调试用）
        RCLCPP_DEBUG(node_->get_logger(), "收到多目标请求: %s", req_data.dump(4).c_str());

        // 2. 验证 tasks 数组是否存在且非空
        if (!req_data.contains("tasks")) {
            result.errCode = 4;
            result.msg = "请求中未包含 tasks 字段";
            return result;
        }
        if (!req_data["tasks"].is_array()) {
            result.errCode = 4;
            result.msg = "tasks 不是数组类型";
            return result;
        }
        const auto& tasks = req_data["tasks"];
        RCLCPP_DEBUG(node_->get_logger(), "tasks 数组大小: %zu", tasks.size());  // 关键日志：确认 tasks 长度
        if (tasks.empty()) {
            result.errCode = 4;
            result.msg = "tasks 数组为空";
            return result;
        }

        // 3. 解析循环参数（保持不变）
        bool loop = req_data.value("loop", false);
        int loop_count = req_data.value("loop_count", 1);
        if (loop && loop_count <= 0) loop_count = 1;

        // 4. 解析目标点列表（添加详细日志）
        std::vector<nav2_msgs::action::NavigateToPose::Goal> goals;
        for (size_t i = 0; i < tasks.size(); ++i) {
            RCLCPP_DEBUG(node_->get_logger(), "开始解析第 %zu 个任务", i);  // 日志：进入循环
            const auto& task = tasks[i];

            // 检查 task 是否为对象
            if (!task.is_object()) {
                throw std::runtime_error("第 " + std::to_string(i) + " 个任务不是对象类型");
            }

            // 验证 task.name
            if (!task.contains("name") || !task["name"].is_string()) {
                throw std::runtime_error("第 " + std::to_string(i) + " 个任务缺少 name 字段");
            }
            std::string task_name = task["name"].get<std::string>();
            if (task_name != "NavigationPositionTask") {
                throw std::runtime_error("第 " + std::to_string(i) + " 个任务 name 应为 NavigationPositionTask，实际为 " + task_name);
            }

            // 验证 start_param 和 destination
            if (!task.contains("start_param") || !task["start_param"].is_object()) {
                throw std::runtime_error("第 " + std::to_string(i) + " 个任务缺少 start_param 字段");
            }
            const auto& start_param = task["start_param"];
            if (!start_param.contains("destination") || !start_param["destination"].is_object()) {
                throw std::runtime_error("第 " + std::to_string(i) + " 个任务缺少 destination 字段");
            }

            // 验证 gridPosition
            const auto& dest = start_param["destination"];
            if (!dest.contains("gridPosition") || !dest["gridPosition"].is_object()) {
                throw std::runtime_error("第 " + std::to_string(i) + " 个任务缺少 gridPosition 字段");
            }
            const auto& grid_pos = dest["gridPosition"];
            if (!grid_pos.contains("x") || !grid_pos["x"].is_number()) {
                throw std::runtime_error("第 " + std::to_string(i) + " 个任务 gridPosition 缺少 x 字段或非数字");
            }
            if (!grid_pos.contains("y") || !grid_pos["y"].is_number()) {
                throw std::runtime_error("第 " + std::to_string(i) + " 个任务 gridPosition 缺少 y 字段或非数字");
            }

            // 验证 angle
            if (!dest.contains("angle") || !dest["angle"].is_string()) {
                throw std::runtime_error("第 " + std::to_string(i) + " 个任务缺少 angle 字段或非字符串");
            }
                // 解析具体数值（添加日志确认解析结果）
                int grid_x = grid_pos["x"].get<int>();
                int grid_y = grid_pos["y"].get<int>();
                std::string angle_str = dest["angle"].get<std::string>();
                double angle_deg;
            try {
                angle_deg = std::stod(angle_str);  // 解析角度字符串
            } catch (const std::invalid_argument& e) {
                RCLCPP_WARN(node_->get_logger(), "第 %zu 个任务 angle 格式错误: %s，跳过", i, e.what());
                continue;
            }
            RCLCPP_DEBUG(node_->get_logger(), "第 %zu 个任务解析成功: x=%d, y=%d, angle=%f°", 
                       i, grid_x, grid_y, angle_deg);  // 确认解析结果

            // 验证地图参数（添加日志）
            const MapParameters map_params = pose_calculator_->get_map_parameters();
            RCLCPP_DEBUG(node_->get_logger(), "地图参数: resolution=%.2f, origin_x=%.2f, origin_y=%.2f",
                       map_params.resolution, map_params.origin_x, map_params.origin_y);
            if (map_params.resolution <= 0) {
                result.errCode = 7;
                result.msg = "地图分辨率无效（<=0），无法转换坐标";
                return result;
            }

            // 构造目标点并添加到 goals
            nav2_msgs::action::NavigateToPose::Goal goal_msg;
            goal_msg.pose.header.frame_id = "map";
            goal_msg.pose.pose.position.x = grid_to_physical_x(grid_x, map_params);
            goal_msg.pose.pose.position.y = grid_to_physical_y(grid_y, map_params);
            goal_msg.pose.pose.position.z = 0.0;

            tf2::Quaternion quat;
            quat.setRPY(0.0, 0.0, angle_deg * M_PI / 180.0);
            goal_msg.pose.pose.orientation = tf2::toMsg(quat);

            goals.push_back(goal_msg);
            RCLCPP_DEBUG(node_->get_logger(), "第 %zu 个目标点已添加到 goals，当前 goals 大小: %zu", 
                       i, goals.size());  // 关键日志：确认 goals 大小变化
        }
        // 5. 检查 goals 是否为空
        const size_t goal_count = goals.size();
        if (goal_count == 0) {
            result.errCode = 4;
            result.msg = "未解析到有效目标点";
            return result;  // 直接返回失败
        }

        // 6. 初始化多目标状态
        {
            std::lock_guard<std::timed_mutex> lock(multi_goal_mutex_);
            multi_goal_state_ = std::make_unique<MultiGoalState>();
            multi_goal_state_->goals = std::move(goals);
            multi_goal_state_->loop = loop;
            multi_goal_state_->remaining_loops = loop ? loop_count : 0;
            multi_goal_state_->current_index = 0;
            multi_goal_state_->is_running = true;
            multi_goal_state_->is_canceled = false;
            multi_goal_state_->current_goal_handle.reset();
        }

        // 7. 启动第一个目标（保持不变）
        execute_next_goal();
        result.successed = true;  // 标记成功
        result.errCode = 0;       // 成功错误码
        result.msg = "多目标导航任务已启动，共 " + std::to_string(goal_count) + " 个目标点";  // 正确消息
        result.data = "";         // 保持空（按需求）
        return result;

    } catch (const nlohmann::json::exception& e) {
        result.errCode = 3;
        result.msg = "JSON解析失败: " + std::string(e.what());
        RCLCPP_ERROR(node_->get_logger(), "JSON解析异常: %s", e.what());  // 日志：捕获JSON异常
    } catch (const std::exception& e) {
        result.errCode = 5;
        result.msg = "处理失败: " + std::string(e.what());
        RCLCPP_ERROR(node_->get_logger(), "处理异常: %s", e.what());  // 日志：捕获其他异常
    }
    result.msg = "未知错误";
    return result;
}
void NavigationGoalHandler::goal_response_callback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle
) {
    std::lock_guard<std::timed_mutex> lock(multi_goal_mutex_);
    
    if (!goal_handle) {
        // 目标被服务器拒绝
        RCLCPP_ERROR(node_->get_logger(), "导航目标发送失败，被服务器拒绝");
        if (multi_goal_state_) {
            multi_goal_state_->current_goal_handle.reset();
        }
    } else {
        // 目标被接受
        RCLCPP_INFO(node_->get_logger(), "导航目标已被服务器接受");
        if (multi_goal_state_) {
            multi_goal_state_->current_goal_handle = goal_handle;
        }
    }
}
// 执行下一个目标点
void NavigationGoalHandler::execute_next_goal() {
    using GoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;

    // 1. 获取当前状态（短锁）
    std::unique_lock<std::timed_mutex> lock(multi_goal_mutex_, std::chrono::milliseconds(100));
    if (!lock.owns_lock() || !multi_goal_state_ || !multi_goal_state_->is_running) {
        RCLCPP_DEBUG(node_->get_logger(), "无有效任务状态，不执行下一个目标");
        return;
    }
    if (multi_goal_state_->is_paused) {
        RCLCPP_DEBUG(node_->get_logger(), "导航暂停中，不执行下一个目标");
        return;
    }

    // 2. 检查目标列表是否为空
    size_t total_goals = multi_goal_state_->goals.size();
    if (total_goals == 0) {
        RCLCPP_INFO(node_->get_logger(), "目标列表为空，结束导航任务");
        multi_goal_state_->is_running = false;
        return;
    }

    // 3. 处理索引越界（循环/结束逻辑）
    if (multi_goal_state_->current_index >= total_goals) {
        if (multi_goal_state_->loop && multi_goal_state_->remaining_loops != 0) {
            // 循环模式：重置索引并减少剩余循环次数
            multi_goal_state_->current_index = 0;
            if (multi_goal_state_->remaining_loops > 0) {
                multi_goal_state_->remaining_loops--;
            }
            RCLCPP_INFO(node_->get_logger(), "进入下一轮循环，剩余循环次数: %d", 
                      multi_goal_state_->remaining_loops);
        } else {
            // 非循环模式：结束任务
            RCLCPP_INFO(node_->get_logger(), "所有目标执行完成，结束导航任务");
            multi_goal_state_->is_running = false;
            return;
        }
    }

    // 4. 拷贝当前目标（解锁前完成，避免锁阻塞）
    size_t current_idx = multi_goal_state_->current_index;
    nav2_msgs::action::NavigateToPose::Goal current_goal = multi_goal_state_->goals[current_idx];
    lock.unlock();  // 提前解锁

    // 5. 发送下一个目标
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(
        &NavigationGoalHandler::goal_response_callback, this, std::placeholders::_1
    );
    send_goal_options.result_callback = [this](const GoalHandle::WrappedResult& result) {
        // 复用上面修复的结果回调逻辑
        std::unique_lock<std::timed_mutex> lock(multi_goal_mutex_, std::chrono::milliseconds(100));
        if (!lock.owns_lock() || !multi_goal_state_ || multi_goal_state_->is_canceled) {
            return;
        }
        if (multi_goal_state_->is_paused) {
            return;
        }
        multi_goal_state_->current_index++;
        multi_goal_state_->current_goal_handle.reset();
        lock.unlock();
        execute_next_goal();
    };

    navigate_client_->async_send_goal(current_goal, send_goal_options);
    RCLCPP_INFO(node_->get_logger(), "已发送第 %zu 个目标（共 %zu 个）", 
              current_idx + 1, total_goals);
}


NavigationResult NavigationGoalHandler::stop_all_tasks() {
    NavigationResult result;
    result.successed = false;
    result.errCode = 0;
    result.msg = "导航任务已停止";

    try {
        std::lock_guard<std::timed_mutex> lock(multi_goal_mutex_);
        if (multi_goal_state_ && multi_goal_state_->is_running) {
            // 1. 标记任务为取消状态
            multi_goal_state_->is_canceled = true;
            multi_goal_state_->is_running = false;

            // 2. 精准取消当前正在执行的目标
            if (multi_goal_state_->current_goal_handle) {
                navigate_client_->async_cancel_goal(
                    multi_goal_state_->current_goal_handle,
                    [this](const action_msgs::srv::CancelGoal::Response::SharedPtr response) {
                        RCLCPP_INFO(node_->get_logger(), "已取消当前执行的目标");
                    }
                );
                multi_goal_state_->current_goal_handle.reset(); // 清空句柄
            }

            // 3. 唤醒所有等待的线程
            multi_goal_cv_.notify_all();
            result.successed = true;
        } else {
            result.msg = "当前无正在执行的导航任务";
            result.successed = true;
        }
    } catch (const std::exception& e) {
        result.errCode = 5;
        result.msg = "停止任务失败: " + std::string(e.what());
    }
    return result;
}
// 取消当前导航目标（调用Nav2动作的取消接口）
void NavigationGoalHandler::cancel_current_goal() {
    if (navigate_client_ && navigate_client_->action_server_is_ready()) {
        // 取消所有未完成的目标
        navigate_client_->async_cancel_all_goals(
            [this](const action_msgs::srv::CancelGoal::Response::SharedPtr response) {
                // 使用标准状态码判断（修正部分）
                if (response->return_code == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
                    RCLCPP_INFO(node_->get_logger(), "成功取消当前导航目标");
                } else if (response->return_code == action_msgs::msg::GoalStatus::STATUS_CANCELED) {
                    RCLCPP_INFO(node_->get_logger(), "导航目标已被取消");
                } else {
                    RCLCPP_WARN(node_->get_logger(), "取消导航目标失败，返回码: %d", response->return_code);
                }
            }
        );
    } else {
        RCLCPP_WARN(node_->get_logger(), "导航动作服务器未就绪，无法取消目标");
    }
}

// 实现状态查询逻辑
NavigationGoalHandler::NavigationStatus NavigationGoalHandler::get_navigation_status() {
    NavigationStatus status;
    status.status = 1; // 默认完成
    status.remaining = 0;
    status.message = "successed";

    std::lock_guard<std::timed_mutex> lock(multi_goal_mutex_);
    if (!multi_goal_state_) {
        // 无任务状态视为完成
        return status;
    }

    // 检查是否已取消
    if (multi_goal_state_->is_canceled) {
        status.status = 2;
        status.message = "cancelled";
        return status;
    }

    // 检查是否正在运行
    if (multi_goal_state_->is_running) {
        status.status = 3;
        // 计算剩余目标点数量
        size_t total = multi_goal_state_->goals.size();
        size_t current = multi_goal_state_->current_index;
        
        // 处理循环情况
        if (multi_goal_state_->loop && multi_goal_state_->remaining_loops > 0) {
            status.remaining = (total - current) + (multi_goal_state_->remaining_loops - 1) * total;
        } else {
            status.remaining = total - current;
        }
        
        // 确保剩余数量不为负
        if (status.remaining > total) {
            status.remaining = 0;
        }
        
        status.message = "当前剩余" + std::to_string(status.remaining) + "个导航目标点";
        return status;
    }
    return status;
}

// 超声波数据回调函数：实时更新最新数据
void NavigationGoalHandler::ultrasound_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ultrasound_mutex_);
    latest_ultrasound_data_ = msg->data;  // 存储原始数据
    RCLCPP_DEBUG(node_->get_logger(), "收到超声波数据，长度: %zu", latest_ultrasound_data_.size());
}

// 障碍物识别核心逻辑：处理数据并生成结果
ObstacleResult NavigationGoalHandler::identify_obstacles() {
    ObstacleResult result;
    result.successed = false;
    result.errCode = 0;
    result.msg = "";
    result.data = "";
    result.raw_data = "";

    try {
        // 1. 获取最新数据（加锁保护）
        std::vector<float> ultrasound_data;
        {
            std::lock_guard<std::mutex> lock(ultrasound_mutex_);
            if (latest_ultrasound_data_.empty()) {
                result.errCode = 6;
                result.msg = "未收到超声波数据";
                return result;
            }
            ultrasound_data = latest_ultrasound_data_;  // 拷贝数据用于处理
        }

        // 2. 处理数据：确保为8个传感器值（不足补0，超过截断）
        std::vector<float> processed_data(8, 0.0f);  // 初始化8个传感器
        size_t copy_size = std::min(ultrasound_data.size(), processed_data.size());
        std::copy(ultrasound_data.begin(), ultrasound_data.begin() + copy_size, processed_data.begin());

        // 3. 生成障碍状态字符串（1有障碍，0无障碍）
        std::string obstacle_status;
        for (float distance : processed_data) {
            // 与0.5m比较：小于则视为有障碍
            obstacle_status += (distance < 0.5f && distance > 0.0f) ? "1" : "0";
        }

        // 4. 生成原始数据字符串（逗号分隔）
        std::string raw_data_str;
        for (size_t i = 0; i < processed_data.size(); ++i) {
            if (i > 0) raw_data_str += ", ";
            raw_data_str += std::to_string(processed_data[i]);  // 可格式化保留2位小数
        }

        // 5. 填充成功结果
        result.successed = true;
        result.errCode = 0;
        result.msg = "success";
        result.data = obstacle_status;
        result.raw_data = raw_data_str;
        RCLCPP_DEBUG(node_->get_logger(), "障碍物识别完成，状态: %s", obstacle_status.c_str());

    } catch (const std::exception& e) {
        result.errCode = 5;
        result.msg = "处理失败: " + std::string(e.what());
    }

    return result;
}

ObstacleResult NavigationGoalHandler::pause_navigation() {
    ObstacleResult result;
    result.successed = false;
    result.errCode = 0;
    result.msg = "未执行任何任务";

    // 阶段1：快速检查状态并标记暂停（最小化锁持有时间）
    {
        std::lock_guard<std::timed_mutex> lock(multi_goal_mutex_);
        if (!multi_goal_state_ || !multi_goal_state_->is_running) {
            return result;
        }
        if (multi_goal_state_->is_paused) {
            result.msg = "已处于暂停状态";
            result.successed = true;
            return result;
        }
        multi_goal_state_->is_paused = true;  // 提前标记暂停
    }

    // 阶段2：后台线程处理耗时操作（缩小锁范围）
    std::thread([this]() {
        // 仅在需要访问共享资源时加锁
        std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle;
        
        // 先获取目标句柄（短时间持有锁）
        {
            std::lock_guard<std::timed_mutex> lock(multi_goal_mutex_);
            if (multi_goal_state_) {
                goal_handle = multi_goal_state_->current_goal_handle;  // 拷贝句柄
            }
        }

        // 1. 取消目标（无锁操作，使用拷贝的句柄）
        if (goal_handle) {
            RCLCPP_INFO(node_->get_logger(), "后台线程：开始取消目标");
            auto cancel_future = navigate_client_->async_cancel_goal(
                goal_handle,
                [this](const action_msgs::srv::CancelGoal::Response::SharedPtr response) {
                    if (response->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE) {
                        RCLCPP_INFO(node_->get_logger(), "后台线程：目标取消成功");
                    } else {
                        RCLCPP_ERROR(node_->get_logger(), "后台线程：取消失败，错误码: %d", response->return_code);
                    }
                }
            );
            // 超时等待（不阻塞锁）
            cancel_future.wait_for(std::chrono::milliseconds(200));
        }

        // 2. 强制停速（无锁操作）
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);
        RCLCPP_INFO(node_->get_logger(), "后台线程：已发布零速度指令");

        // 3. 重置句柄（短时间持有锁）
        {
            std::lock_guard<std::timed_mutex> lock(multi_goal_mutex_);
            if (multi_goal_state_) {
                multi_goal_state_->current_goal_handle.reset();
            }
        }
    }).detach();

    // 立即返回响应
    result.successed = true;
    result.msg = "暂停请求已接收，正在处理";
    return result;
}

ObstacleResult NavigationGoalHandler::resume_navigation() {
    ObstacleResult result;
    result.successed = false;
    result.msg = "无暂停的任务";

    // 关键修复：使用try_lock带超时，避免死等锁
    std::unique_lock<std::timed_mutex> lock(multi_goal_mutex_, std::chrono::milliseconds(100));
    if (!lock.owns_lock()) {  // 超时未获取锁
        result.msg = "获取状态失败，可能有其他操作阻塞";
        return result;
    }

    // 检查状态
    if (!multi_goal_state_ || !multi_goal_state_->is_running) {
        return result;
    }
    if (!multi_goal_state_->is_paused) {
        result.msg = "任务未暂停，无需恢复";
        result.successed = true;
        return result;
    }

    // 重置暂停状态
    multi_goal_state_->is_paused = false;
    lock.unlock();  // 提前解锁，减少阻塞

    // 重新执行任务（无锁触发）
    execute_next_goal();

    result.successed = true;
    result.msg = "导航已恢复";
    return result;
}


ObstacleResult NavigationGoalHandler::set_init_pose(const nlohmann::json& req_data) {
    ObstacleResult result;
    result.successed = false;
    result.errCode = 0;
    result.msg = "";
    result.data = "null";
    result.raw_data = "";

    try {
        // 1. 验证必填入参
        if (!req_data.contains("gridX") || !req_data.contains("gridY")) {
            result.errCode = 4;
            result.msg = "请求缺少 gridX 或 gridY 字段";
            return result;
        }
        if (!req_data["gridX"].is_number() || !req_data["gridY"].is_number()) {
            result.errCode = 4;
            result.msg = "gridX 或 gridY 不是数字类型";
            return result;
        }

        // 2. 解析栅格坐标和角度
        int grid_x = req_data["gridX"].get<int>();
        int grid_y = req_data["gridY"].get<int>();
        double angle_deg = req_data.value("angle", 0.0);  // 默认为0度
        std::string map_name = req_data.value("mapName", "");  // 地图名称（可选）
        int type = req_data.value("type", 0);  // 类型（可选，暂不处理）

        RCLCPP_DEBUG(node_->get_logger(), "解析入参: gridX=%d, gridY=%d, angle=%f°, mapName=%s",
                   grid_x, grid_y, angle_deg, map_name.c_str());

        // 3. 验证地图参数
        const MapParameters map_params = pose_calculator_->get_map_parameters();
        if (map_params.resolution <= 0) {
            result.errCode = 7;
            result.msg = "地图分辨率无效，无法转换坐标";
            return result;
        }
        // 可选：验证地图名称是否匹配（暂时不用需要）
        // if (!map_name.empty() && map_name != map_params.map_name) {
        //     RCLCPP_WARN(node_->get_logger(), "地图名称不匹配（请求: %s, 当前: %s）",
        //               map_name.c_str(), map_params.map_name.c_str());
        //     // 此处可选择返回错误或继续执行，根据需求调整
        //     // result.errCode = 8;
        //     // result.msg = "地图名称不匹配";
        //     // return result;
        // }

        // 4. 栅格坐标转物理坐标
        double x = grid_to_physical_x(grid_x, map_params);
        double y = grid_to_physical_y(grid_y, map_params);
        double yaw = angle_deg * M_PI / 180.0;  // 角度转弧度

        // 5. 发布初始位姿消息（兼容Nav2）
        geometry_msgs::msg::PoseWithCovarianceStamped init_pose_msg;
        init_pose_msg.header.frame_id = "map";
        init_pose_msg.header.stamp = node_->now();
        init_pose_msg.pose.pose.position.x = x;
        init_pose_msg.pose.pose.position.y = y;
        init_pose_msg.pose.pose.position.z = 0.0;

        // 设置朝向（四元数）
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, yaw);  // 仅绕z轴旋转
        init_pose_msg.pose.pose.orientation = tf2::toMsg(quat);

        // 设置置信度 covariance
        init_pose_msg.pose.covariance[0] = 0.25;   // x方差
        init_pose_msg.pose.covariance[7] = 0.25;   // y方差
        init_pose_msg.pose.covariance[35] = 0.0685; // yaw方差

        init_pose_pub_->publish(init_pose_msg);
        RCLCPP_INFO(node_->get_logger(), "已发布初始坐标: (x=%.2f, y=%.2f, angle=%.2f°)", x, y, angle_deg);

        // 6. 构造指定回参
        result.successed = true;
        result.errCode = 0;
        result.msg = "正在使用地图坐标初始化....";  // 按需求固定消息
        return result;

    } catch (const nlohmann::json::exception& e) {
        result.errCode = 3;
        result.msg = "JSON解析失败: " + std::string(e.what());
    } catch (const std::exception& e) {
        result.errCode = 5;
        result.msg = "设置失败: " + std::string(e.what());
    }
    return result;
}


void NavigationGoalHandler::handle_robot_hardware_status(
    const http::request<http::string_body>& req,
    http::response<http::dynamic_body>& res 
) {
    res.set(http::field::content_type, "application/json");
    json response;

    try {
        // 构造固定硬件状态数据（保持不变）
        json data;
        data["anticollision"] = 0;
        data["battery_percentage"] = 66;
        data["camera_num"] = 1;
        data["cameras"] = 0;
        data["cameras_obs"] = 0;
        data["charge"] = 0;
        data["current_map_name"] = "";
        data["current_working"] = 3;
        data["emergency_stop"] = false;
        data["type"] = 1;
        data["ultrasonic"] = 1;
        data["ultrasonic_num"] = 8;

        response["data"] = data;
        response["errCode"] = 0;
        response["msg"] = "success";
        response["successed"] = true;
        res.result(http::status::ok);
    } catch (const std::exception& e) {
        response["data"] = json::object();
        response["errCode"] = 500;
        response["msg"] = "获取硬件状态失败: " + std::string(e.what());
        response["successed"] = false;
        res.result(http::status::internal_server_error);
    }

    // 适配 dynamic_body：使用 ostream 写入响应体（与参考接口一致）
    boost::beast::ostream(res.body()) << response.dump(4);
    res.prepare_payload();
}

void NavigationGoalHandler::handle_robot_local_status(
    const http::request<http::string_body>& req,
    http::response<http::dynamic_body>& res
) {
    res.set(http::field::content_type, "application/json");
    nlohmann::json response;

    try {
        // 返回固定默认值：定位成功
        response["data"] = true;  // 定位成功
        response["errCode"] = 0;
        response["msg"] = "数据获取成功";
        response["successed"] = true;
        res.result(http::status::ok);  // 200 OK
    } catch (const std::exception& e) {
        response["data"] = false;  // 定位失败
        response["errCode"] = 500;
        response["msg"] = "获取定位状态失败: " + std::string(e.what());
        response["successed"] = false;
        res.result(http::status::internal_server_error);
    }

    boost::beast::ostream(res.body()) << response.dump(4);
    res.prepare_payload();
}