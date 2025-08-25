#include "ros2_beast_bridge/http_server.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <sstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <jwt-cpp/jwt.h>

using json = nlohmann::json;

HttpServer::HttpServer(const rclcpp::NodeOptions& options)
    : Node("http_bridge_server", options), 
      port_(8083),
      imu_data_(json::object()),
      pointcloud_metadata_(json::object()),
      string_data_("{}"),
      ioc_(4)  // 初始化io_context，参数为线程数
{
    // 替换配置文件读取
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("ros2_beast_bridge");
    std::string config_path = package_share_dir + "/config/config.json";  // 假设配置文件放在包的share/config目录下
    // 在HttpServer构造函数中添加
    RCLCPP_INFO(this->get_logger(), "尝试读取配置文件: %s", config_path.c_str());
    std::ifstream config_file(config_path);
    if (config_file.is_open()) {
        try {
            json config;
            config_file >> config;

            // 1. 解析jwt_secret（已有逻辑）
            if (config.contains("jwt_secret")) {
                jwt_secret_ = config["jwt_secret"].get<std::string>();
                RCLCPP_INFO(this->get_logger(), "成功读取JWT密钥");
            } else {
                RCLCPP_ERROR(this->get_logger(), "配置文件缺少jwt_secret");
                throw std::runtime_error("missing jwt_secret");
            }

            // 2. 解析http_server_port（已有逻辑）
            if (config.contains("http_server_port")) {
                port_ = config["http_server_port"].get<unsigned short>();
            }

            // 3. 解析token_expire_hours（已有逻辑）
            if (config.contains("token_expire_hours")) {
                token_expire_hours_ = config["token_expire_hours"].get<int>();
            } else {
                token_expire_hours_ = 24;
            }

            // 4. 新增：解析users列表（添加在这里）
            if (config.contains("users")) {
                auto users = config["users"];
                for (const auto& user : users) {
                    // 确保user对象包含usercode和password字段
                    if (user.contains("usercode") && user.contains("password")) {
                        std::string usercode = user["usercode"].get<std::string>();
                        std::string password = user["password"].get<std::string>();
                        valid_users_[usercode] = password;  // 存储到私有成员valid_users_中
                        RCLCPP_INFO(this->get_logger(), "加载用户: %s", usercode.c_str());
                    } else {
                        RCLCPP_WARN(this->get_logger(), "用户配置缺少usercode或password，已跳过");
                    }
                }
                RCLCPP_INFO(this->get_logger(), "成功加载 %zu 个用户", valid_users_.size());
            } else {
                RCLCPP_WARN(this->get_logger(), "配置文件中未找到users字段，无可用用户");
            }

            RCLCPP_INFO(this->get_logger(), "配置文件加载成功，端口: %d", port_);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "解析配置文件失败: %s", e.what());
            throw;  // 抛出异常，终止节点启动
        }
    } else {
        RCLCPP_FATAL(this->get_logger(), "无法打开配置文件: %s", config_path.c_str());
        throw std::runtime_error("config file not found");
    }
    // 初始化命令发布者
    command_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "/http_commands", 10);
        
    // 初始化IMU订阅者
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 10,
        std::bind(&HttpServer::imuCallback, this, std::placeholders::_1));
        
    // 初始化点云订阅者
    pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar", 10,
        std::bind(&HttpServer::pointCloudCallback, this, std::placeholders::_1));
        
    // 初始化字符串订阅者
    string_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/status_string", 10,
        std::bind(&HttpServer::stringCallback, this, std::placeholders::_1));


}
void HttpServer::post_init() {
    // 此时 HttpServer 已被 shared_ptr 管理，可安全调用 shared_from_this()
    pose_calculator_ = std::make_unique<BaseLinkPoseCalculator>(shared_from_this());
    RCLCPP_INFO(this->get_logger(), "BaseLinkPoseCalculator 初始化完成");
    nav_handler_ = std::make_shared<NavigationGoalHandler>(
        shared_from_this(),  // 传递节点指针
        pose_calculator_     // 传递地图参数计算器
    );
        // 启动HTTP服务器线程
    server_thread_ = std::thread(&HttpServer::startServer, this);
    ioc_thread_ = std::thread([this]() {
        ioc_.run();  // 运行Asio事件循环，处理I/O操作
    });

    RCLCPP_INFO(this->get_logger(), "HTTP server started on port %d", port_);
}
std::string HttpServer::generateToken(const std::string& username) {
    auto token = jwt::create()
        .set_issuer("ros2_beast_bridge")
        .set_subject(username)
        .set_issued_at(std::chrono::system_clock::now())
        .set_expires_at(std::chrono::system_clock::now() + std::chrono::hours(token_expire_hours_))  // 使用配置的有效期
        .sign(jwt::algorithm::hs256{jwt_secret_});  // 使用配置的密钥
    return token;
}

bool HttpServer::validateToken(const std::string& token) {
    try {
        auto decoded = jwt::decode(token);
        auto verifier = jwt::verify()
            .allow_algorithm(jwt::algorithm::hs256{jwt_secret_})  // 旧版本API
            .with_issuer("ros2_beast_bridge");
        verifier.verify(decoded);
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

// IMU数据回调
void HttpServer::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (!msg) {  // 检查msg是否为空
        RCLCPP_WARN(this->get_logger(), "收到空的IMU消息");
        return;
    }
    try {
        // 正确构建JSON对象
        json imu_json = {
            {"header", {
                {"frame_id", msg->header.frame_id},
                {"stamp", {
                    {"sec", msg->header.stamp.sec},
                    {"nanosec", msg->header.stamp.nanosec}
                }}
            }},
            {"orientation", {
                {"x", msg->orientation.x},
                {"y", msg->orientation.y},
                {"z", msg->orientation.z},
                {"w", msg->orientation.w}
            }},
            {"angular_velocity", {
                {"x", msg->angular_velocity.x},
                {"y", msg->angular_velocity.y},
                {"z", msg->angular_velocity.z}
            }},
            {"linear_acceleration", {
                {"x", msg->linear_acceleration.x},
                {"y", msg->linear_acceleration.y},
                {"z", msg->linear_acceleration.z}
            }},
            {"timestamp", std::time(nullptr)}  // 添加时间戳，方便前端判断数据新鲜度
        };

        // 线程安全地更新共享数据
        std::lock_guard<std::mutex> lock(imu_mutex_);
        imu_data_ = imu_json;  // 确保imu_data_是类成员变量，类型为json
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "IMU数据转换失败: %s", e.what());
    }
}

// 点云数据回调（只存储元数据）
void HttpServer::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    pointcloud_metadata_ = {
        {"header", {
            {"stamp_sec", msg->header.stamp.sec},  // 秒级时间戳
            {"stamp_nanosec", msg->header.stamp.nanosec},  // 纳秒部分
            {"frame_id", msg->header.frame_id}
        }},
        {"width", msg->width},
        {"height", msg->height},
        {"point_step", msg->point_step},
        {"row_step", msg->row_step},
        {"is_dense", msg->is_dense},
        {"data_size", msg->data.size()}
    };
}

// 字符串数据回调
void HttpServer::stringCallback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    string_data_ = msg->data;
}
HttpServer::~HttpServer() {
    ioc_.stop();  
    if(ioc_thread_.joinable()){
        ioc_thread_.join();
    }
    if (server_thread_.joinable()) {
        server_thread_.join(); 
    }
    RCLCPP_INFO(this->get_logger(), "HTTP server stopped");
}

// 修改startServer函数中的线程创建部分
void HttpServer::startServer() {
    try {
        auto const address = net::ip::make_address("0.0.0.0");
        tcp::acceptor acceptor{ioc_, {address, port_}};
        acceptor.set_option(net::socket_base::reuse_address(true));
        
        while (rclcpp::ok()) {
            tcp::socket socket{ioc_};
            acceptor.accept(socket);
            
            // 使用lambda捕获this指针和socket
            std::thread{
                [this, sock = std::move(socket)]() mutable {
                    this->handleSession(std::move(sock));
                }
            }.detach();
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Server error: %s, restarting...", e.what());
        std::this_thread::sleep_for(std::chrono::seconds(1));
        startServer();
    }
}


void HttpServer::handleSession(tcp::socket socket) {
    try {
        beast::flat_buffer buffer;
        http::request<http::string_body> req;
        http::read(socket, buffer, req);
        
        http::response<http::dynamic_body> res;
        processRequest(req, res);
        
        http::write(socket, res);
        socket.shutdown(tcp::socket::shutdown_send);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Session error: %s", e.what());
    }
}
void HttpServer::processRequest(
    const http::request<http::string_body>& req,
    http::response<http::dynamic_body>& res) 
{
    res.version(req.version());
    res.set(http::field::server, "ROS2-HTTP-Bridge");
    res.set(http::field::access_control_allow_origin, "*");
    res.set(http::field::access_control_allow_methods, "GET, POST, OPTIONS");
    res.set(http::field::access_control_allow_headers, "Content-Type, Authorization");  // 新增Authorization头

    // 处理预检请求
    if (req.method() == http::verb::options) {
        res.result(http::status::ok);
        return;
    }
    
    try {
        // 1. 登录接口（无需Token验证）
        if (req.method() == http::verb::post && req.target() == "/auth/token") {
            auto j = json::parse(req.body());
            std::string usercode = j["userCode"];  // 匹配前端传递的usercode
            std::string password = j["password"];

            RCLCPP_INFO(this->get_logger(), "收到登录请求：用户 %s", usercode.c_str());

            // 验证用户名和密码
            if (valid_users_.count(usercode) && valid_users_[usercode] == password) {
                // 生成Token
                std::string token = generateToken(usercode);
                RCLCPP_INFO(this->get_logger(), "用户 %s 登录成功", usercode.c_str());

                // 构建成功响应
                res.set(http::field::content_type, "application/json");
                res.result(http::status::ok);
                json response = {
                    {"status", "success"},
                    {"message", "登录成功"},
                    {"token", token},
                    {"expires_in", token_expire_hours_ * 3600}  // 有效期（秒）
                };
                beast::ostream(res.body()) << response.dump();
            } else {
                // 验证失败
                RCLCPP_WARN(this->get_logger(), "用户 %s 登录失败：用户名或密码错误", usercode.c_str());
                res.set(http::field::content_type, "application/json");
                res.result(http::status::unauthorized);
                json response = {
                    {"status", "error"},
                    {"error", "用户名或密码错误"}
                };
                beast::ostream(res.body()) << response.dump();
            }
            return;  // 处理完登录请求后返回
        }

        // 2. 验证Token（除登录外的接口均需验证）
        // 需要保护的接口列表
        const std::vector<std::string> protected_targets = {
            "/buaacar/imu",
            "/buaacar/pointcloud",
            "/buaacar/string",
            "/command",
            "/task_queue/identify_obstacles",
            "/real_time_data/robot_hardware_status",
            "/real_time_data/robot_pos",
            "/real_time_data/temp_point_init_pose",
            "/real_time_data/robot_local_status",
            "/real_time_data/map",
            "/task_queue/one_point",
            "/real_time_data/local_grid_plan_path",
            "/real_time_data/two_point_plan_path",
            "/task_queue/start",
            "/task_queue/stop",
            "/task_queue/task/is_finished",
            "/task_queue/pause",
            "/task_queue/resume"
        };

        // 检查当前请求是否需要验证Token
        bool need_auth = std::find(protected_targets.begin(), protected_targets.end(), std::string(req.target())) != protected_targets.end();

        // 修改processRequest函数中的Token验证部分
        if (need_auth) {
            // 从请求头获取Token
            auto auth_header = req.find(http::field::authorization);
            if (auth_header == req.end()) {  // 检查头是否存在
                res.set(http::field::content_type, "application/json");
                res.result(http::status::unauthorized);
                json response = {{"error", "未提供Token，请先登录"}};
                beast::ostream(res.body()) << response.dump();
                return;
            }

            // 解析Token（格式：Bearer <token>）
            std::string auth_str = std::string(auth_header->value());
            if (auth_str.substr(0, 7) != "Bearer ") {
            res.set(http::field::content_type, "application/json");
            res.result(http::status::unauthorized);
            json response = {{"error", "Token格式错误，应为Bearer <token>"}};
            beast::ostream(res.body()) << response.dump();
            return;
            }
            std::string token = auth_str.substr(7);

            // 验证Token有效性
            if (!validateToken(token)) {
            res.set(http::field::content_type, "application/json");
            res.result(http::status::unauthorized);
            json response = {{"error", "Token无效或已过期，请重新登录"}};
            beast::ostream(res.body()) << response.dump();
            return;
            }
        }


        // 3. 处理受保护的接口（已通过Token验证）
        // 处理POST命令
        if (req.method() == http::verb::post && req.target() == "/command") {
            auto j = json::parse(req.body());
            std::string cmd = j["command"];
            
            auto msg = std_msgs::msg::String();
            msg.data = cmd;
            command_publisher_->publish(msg);
            
            res.set(http::field::content_type, "application/json");
            res.result(http::status::ok);
            json response_json = {{"status", "success"}, {"command", cmd}};
            beast::ostream(res.body()) << response_json.dump();
        }
        // 提供网页界面（无需验证）
        else if (req.method() == http::verb::get && req.target() == "/") {
            std::string package_share_dir = ament_index_cpp::get_package_share_directory("ros2_beast_bridge");
            std::string web_path = package_share_dir + "/web/index.html";
            
            std::ifstream file(web_path);
            if(file) {
                res.set(http::field::content_type, "text/html");
                res.result(http::status::ok);
                beast::ostream(res.body()) << std::string(
                    (std::istreambuf_iterator<char>(file)),
                    std::istreambuf_iterator<char>());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", web_path.c_str());
                res.result(http::status::not_found);
            }
        }
        // 获取IMU数据
        else if (req.method() == http::verb::get && req.target() == "/buaacar/imu") {
             std::lock_guard<std::mutex> lock(imu_mutex_); 
            res.set(http::field::content_type, "application/json");
            if (imu_data_.empty()) {
                beast::ostream(res.body()) << R"({"error": "no IMU data received yet"})";
            } else {
                beast::ostream(res.body()) << imu_data_.dump();
            }
            res.result(http::status::ok);
        }
        // 获取点云元数据
        else if (req.method() == http::verb::get && req.target() == "/buaacar/pointcloud") {
            std::lock_guard<std::mutex> lock(data_mutex_);
            res.set(http::field::content_type, "application/json");
            if (pointcloud_metadata_.empty()) {
                beast::ostream(res.body()) << R"({"error": "no pointcloud data received yet"})";
            } else {
                beast::ostream(res.body()) << pointcloud_metadata_.dump();
            }
            res.result(http::status::ok);
        }
        // 获取字符串数据
        else if (req.method() == http::verb::get && req.target() == "/buaacar/string") {
            std::lock_guard<std::mutex> lock(data_mutex_);
            res.set(http::field::content_type, "text/plain"); 
            res.result(http::status::ok);
            beast::ostream(res.body()) << string_data_; 
        }
        // 获取base_link_pose
        else if (req.method() == http::verb::get && req.target() == "/real_time_data/robot_pos") {
            auto grid_pose = pose_calculator_->get_current_grid_pose();

            res.set(http::field::content_type, "application/json");
            if (grid_pose) {
                // 构造外层响应对象
                json response;
                // 数据部分
                json data;
                data["angle"] = grid_pose->yaw_deg;  // 机器人朝向角（°）
                
                // 栅格坐标子对象
                json grid_position;
                grid_position["x"] = grid_pose->grid_x;
                grid_position["y"] = grid_pose->grid_y;
                data["gridPosition"] = grid_position;  // 嵌套栅格坐标
                
                // 组装完整响应
                response["data"] = data;
                response["errCode"] = 0;               // 成功状态码
                response["msg"] = "数据获取成功";      // 成功消息
                response["successed"] = true;          // 成功标识

                res.result(http::status::ok);
                boost::beast::ostream(res.body()) << response.dump(4);  // 格式化输出JSON
            } else {
                // 错误响应格式
                json response;
                response["data"] = {};                 // 空数据
                response["errCode"] = 1;               // 错误状态码
                response["msg"] = "数据获取失败（可能地图未加载或TF变换异常）";
                response["successed"] = false;         // 失败标识

                res.result(http::status::ok);
                boost::beast::ostream(res.body()) << response.dump(4);
            }
            res.prepare_payload();
            return;
        }
        else if (req.method() == http::verb::get && req.target() == "/real_time_data/map") {
            res.set(http::field::content_type, "application/json");
            
            if (pose_calculator_->is_map_loaded()) {
                auto map_params = pose_calculator_->get_map_parameters();
                
                json response;
                json data;
                json map_info;
                
                // 基础地图信息
                map_info["height"] = map_params.height;          // 地图高度（像素数）
                map_info["width"] = map_params.width;            // 地图宽度（像素数）
                map_info["resolution"] = map_params.resolution;  // 分辨率（米/像素）
                
                // 地图原点坐标（物理坐标，米）
                json map_origin;
                map_origin["x"] = map_params.origin_x;
                map_origin["y"] = map_params.origin_y;
                map_info["origin"] = map_origin;
                                
                data["mapInfo"] = map_info;
                response["data"] = data;
                response["errCode"] = 0;
                response["msg"] = "地图数据获取成功";
                response["successed"] = true;
                
                res.result(http::status::ok);
                boost::beast::ostream(res.body()) << response.dump(4);
            } else {
                json response;
                response["data"] = {};
                response["errCode"] = 2;
                response["msg"] = "地图数据未加载";
                response["successed"] = false;
                
                res.result(http::status::ok);
                boost::beast::ostream(res.body()) << response.dump(4);
            }
            
            res.prepare_payload();
            return;
        }
        else if (req.method() == http::verb::post && req.target() == "/task_queue/one_point") {
            res.set(http::field::content_type, "application/json");
            nlohmann::json response;

            try {
                // 解析请求体
                const auto req_data = nlohmann::json::parse(req.body());
                
                // 调用导航处理器处理请求
                const auto nav_result = nav_handler_->handle_goal(req_data);

                // 构造响应
                response["data"]["gridPhits"] = nlohmann::json::array();
                for (const auto& grid : nav_result.grid_phits) {
                    response["data"]["gridPhits"].push_back({
                        {"x", grid.first},
                        {"y", grid.second}
                    });
                }
                response["errCode"] = nav_result.errCode;
                response["msg"] = nav_result.msg;
                response["successed"] = nav_result.successed;

                res.result(nav_result.successed ? http::status::ok : http::status::bad_request);
            } catch (const nlohmann::json::parse_error& e) {
                response["data"] = {};
                response["errCode"] = 3;
                response["msg"] = "请求格式错误（JSON无效）: " + std::string(e.what());
                response["successed"] = false;
                res.result(http::status::bad_request);
            }

            boost::beast::ostream(res.body()) << response.dump(4);
            res.prepare_payload();
            return;
        }
        else if (req.method() == http::verb::get && req.target() == "/real_time_data/local_grid_plan_path") {
            res.set(http::field::content_type, "application/json");
            nlohmann::json response;

            try {
                // 1. 检查地图是否加载
                if (!pose_calculator_->is_map_loaded()) {
                    response["data"] = {};
                    response["errCode"] = 2;
                    response["msg"] = "地图未加载，无法获取路径";
                    response["successed"] = false;
                    res.result(http::status::ok);
                    boost::beast::ostream(res.body()) << response.dump(4);
                    return;
                }

                // 2. 获取最新路径（/plan话题的实时剩余路径）和地图参数
                auto map_params = pose_calculator_->get_map_parameters();
                std::lock_guard<std::mutex> lock(nav_handler_->get_path_mutex());  // 线程安全
                const auto& remaining_path = nav_handler_->get_latest_path();  // 直接使用/plan的最新数据

                if (remaining_path.poses.empty()) {
                    response["data"] = {};
                    response["errCode"] = 8;
                    response["msg"] = "当前无剩余导航路径（可能已到达目标或未开始导航）";
                    response["successed"] = false;
                    res.result(http::status::ok);
                    boost::beast::ostream(res.body()) << response.dump(4);
                    return;
                }

                // 3. 将剩余路径的物理坐标转为栅格坐标（统一左上角原点）
                nlohmann::json grid_phits_json = nlohmann::json::array();
                for (const auto& pose_stamped : remaining_path.poses) {
                    const auto& pos = pose_stamped.pose.position;
                    // 使用导航处理器中已统一的坐标转换函数
                    int grid_x = nav_handler_->physical_to_grid_x(pos.x, map_params);
                    int grid_y = nav_handler_->physical_to_grid_y(pos.y, map_params);
                    grid_phits_json.push_back({
                        {"x", grid_x},
                        {"y", grid_y}
                    });
                }

                // 4. 构造响应
                response["data"]["gridPhits"] = grid_phits_json;
                response["errCode"] = 0;
                response["msg"] = "剩余路径获取成功";
                response["successed"] = true;
                res.result(http::status::ok);
                boost::beast::ostream(res.body()) << response.dump(4);
            } catch (const std::exception& e) {
                response["data"] = {};
                response["errCode"] = 5;
                response["msg"] = "处理失败: " + std::string(e.what());
                response["successed"] = false;
                res.result(http::status::internal_server_error);
                boost::beast::ostream(res.body()) << response.dump(4);
            }

            res.prepare_payload();
            return;
        }
        else if (req.method() == http::verb::post && req.target() == "/real_time_data/two_point_plan_path") {
            res.set(http::field::content_type, "application/json");
            nlohmann::json response;

            try {
                // 1. 解析请求体
                const auto req_data = nlohmann::json::parse(req.body());
                
                // 2. 调用导航处理器规划路径
                const auto plan_result = nav_handler_->plan_path(req_data);

                // 3. 构造响应
                response["data"]["gridPhits"] = nlohmann::json::array();
                for (const auto& grid : plan_result.grid_phits) {
                    response["data"]["gridPhits"].push_back({
                        {"x", grid.first},
                        {"y", grid.second}
                    });
                }
                response["errCode"] = plan_result.errCode;
                response["msg"] = plan_result.msg;
                response["successed"] = plan_result.successed;

                res.result(plan_result.successed ? http::status::ok : http::status::bad_request);
            } catch (const nlohmann::json::parse_error& e) {
                response["data"] = {};
                response["errCode"] = 3;
                response["msg"] = "请求格式错误（JSON无效）: " + std::string(e.what());
                response["successed"] = false;
                res.result(http::status::bad_request);
            }

            boost::beast::ostream(res.body()) << response.dump(4);
            res.prepare_payload();
            return;
        }
        else if (req.method() == http::verb::post && req.target() == "/task_queue/start") {
            res.set(http::field::content_type, "application/json");
            nlohmann::json response;

            try {
                // 解析请求体
                const auto req_data = nlohmann::json::parse(req.body());
                
                // 调用导航处理器处理多目标请求
                const auto multi_result = nav_handler_->handle_multi_goals(req_data);

                // 构造响应（按需求返回空data）
                response["data"] = "";
                response["errCode"] = multi_result.errCode;
                response["msg"] = multi_result.msg;
                response["successed"] = multi_result.successed;

                res.result(multi_result.successed ? http::status::ok : http::status::bad_request);
            } catch (const nlohmann::json::parse_error& e) {
                response["data"] = "";
                response["errCode"] = 3;
                response["msg"] = "请求格式错误（JSON无效）: " + std::string(e.what());
                response["successed"] = false;
                res.result(http::status::bad_request);
            }

            boost::beast::ostream(res.body()) << response.dump(4);
            res.prepare_payload();
            return;
        }
        else if (req.method() == http::verb::post && req.target() == "/task_queue/stop") {
            res.set(http::field::content_type, "application/json");
            nlohmann::json response;

            try {
                // 调用停止任务方法（入参为空，直接处理）
                const auto stop_result = nav_handler_->stop_all_tasks();

                response["data"] = nullptr;
                response["errCode"] = stop_result.errCode;
                response["msg"] = stop_result.msg;
                response["successed"] = stop_result.successed;
                res.result(http::status::ok);
            } catch (const std::exception& e) {
                response["data"] = nullptr;
                response["errCode"] = 5;
                response["msg"] = "处理失败: " + std::string(e.what());
                response["successed"] = false;
                res.result(http::status::internal_server_error);
            }

            boost::beast::ostream(res.body()) << response.dump(4);
            res.prepare_payload();
            return;
        }
        else if (req.method() == http::verb::get && req.target() == "/task_queue/task/is_finished") {
            res.set(http::field::content_type, "application/json");
            nlohmann::json response;

            try {
                auto status = nav_handler_->get_navigation_status();

                // 关键：按指定顺序手动插入字段
                response["status"] = std::to_string(status.status);       // 第1个字段
                response["errCode"] = 0;                                  // 第2个字段
                response["msg"] = status.message;                         // 第3个字段
                response["successed"] = true;                             // 第4个字段

                res.result(http::status::ok);
            } catch (const std::exception& e) {
                // 异常情况下也保持相同顺序
                response["status"] = "0";
                response["errCode"] = 5;
                response["msg"] = "查询失败: " + std::string(e.what());
                response["successed"] = false;
                res.result(http::status::internal_server_error);
            }

            boost::beast::ostream(res.body()) << response.dump(4);
            res.prepare_payload();
            return;
        }
        else if (req.method() == http::verb::post && req.target() == "/task_queue/identify_obstacles") {
            res.set(http::field::content_type, "application/json");
            nlohmann::json response;

            try {
                // 调用障碍物识别逻辑
                ObstacleResult obstacle_result = nav_handler_->identify_obstacles();

                // 构造JSON回参
                response["data"] = obstacle_result.data;
                response["raw_data"] = obstacle_result.raw_data;
                response["errCode"] = obstacle_result.errCode;
                response["msg"] = obstacle_result.msg;
                response["successed"] = obstacle_result.successed;
                res.result(http::status::ok);
            } catch (const std::exception& e) {
                response["errCode"] = 5;
                response["msg"] = "接口处理失败: " + std::string(e.what());
                response["successed"] = false;
                res.result(http::status::internal_server_error);
            }

            boost::beast::ostream(res.body()) << response.dump(4);
            res.prepare_payload();
            return;
        }
        else if (req.method() == http::verb::post && req.target() == "/task_queue/pause") {
            res.set(http::field::content_type, "application/json");
            nlohmann::json response;

            ObstacleResult result = nav_handler_->pause_navigation();
            response["data"] = nullptr;  // 固定为null
            response["errCode"] = result.errCode;
            response["msg"] = result.msg;
            response["successed"] = result.successed;

            boost::beast::ostream(res.body()) << response.dump(4);
            res.prepare_payload();
            return;
        }
        else if (req.method() == http::verb::post && req.target() == "/task_queue/resume") {
            res.set(http::field::content_type, "application/json");
            nlohmann::json response;

            ObstacleResult result = nav_handler_->resume_navigation();
            response["data"] = nullptr;  // 固定为null
            response["errCode"] = result.errCode;
            response["msg"] = result.msg;
            response["successed"] = result.successed;

            try {
                ObstacleResult result = nav_handler_->resume_navigation();
                response["data"] = nullptr;
                response["errCode"] = result.errCode;
                response["msg"] = result.msg;
                response["successed"] = result.successed;
                res.result(http::status::ok);
            } catch (const std::exception& e) {
                response["msg"] = "恢复失败: " + std::string(e.what());
                response["successed"] = false;
                res.result(http::status::internal_server_error);
            }

            boost::beast::ostream(res.body()) << response.dump(4);
            res.prepare_payload();
            return;
        }
        else if (req.method() == http::verb::post && req.target() == "/real_time_data/temp_point_init_pose") {
            res.set(http::field::content_type, "application/json");
            nlohmann::json response;

            try {
                // 解析请求数据
                nlohmann::json req_data = nlohmann::json::parse(req.body());
                ObstacleResult result = nav_handler_->set_init_pose(req_data);

                // 构造回参
                response["data"] = nullptr;
                response["errCode"] = result.errCode;
                response["msg"] = result.msg;
                response["successed"] = result.successed;
                res.result(http::status::ok);
            } catch (const nlohmann::json::exception& e) {
                response["data"] = nullptr;
                response["errCode"] = 3;
                response["msg"] = "请求格式错误: " + std::string(e.what());
                response["successed"] = false;
                res.result(http::status::bad_request);
            } catch (const std::exception& e) {
                response["data"] = nullptr;
                response["errCode"] = 5;
                response["msg"] = "接口处理失败: " + std::string(e.what());
                response["successed"] = false;
                res.result(http::status::internal_server_error);
            }

            boost::beast::ostream(res.body()) << response.dump(4);
            res.prepare_payload();
            return;
        }
        else if (req.method() == http::verb::get && req.target() == "/real_time_data/robot_hardware_status") {
            res.set(http::field::content_type, "application/json");
            try {
                // 直接调用处理函数，传入 req 和 res（类型已匹配）
                nav_handler_->handle_robot_hardware_status(req, res);
            } catch (const std::exception& e) {
                // 异常处理（与参考接口格式一致）
                nlohmann::json response;
                response["data"] = nullptr;
                response["errCode"] = 5;
                response["msg"] = "硬件状态接口处理失败: " + std::string(e.what());
                response["successed"] = false;
                res.result(http::status::internal_server_error);
                boost::beast::ostream(res.body()) << response.dump(4);
                res.prepare_payload();
            }
            return;
        }
        else if (req.method() == http::verb::get && req.target() == "/real_time_data/robot_local_status") {
            res.set(http::field::content_type, "application/json");
            try {
                // 调用 NavigationGoalHandler 的处理函数
                nav_handler_->handle_robot_local_status(req, res);
            } catch (const std::exception& e) {
                nlohmann::json response;
                response["data"] = false;
                response["errCode"] = 5;
                response["msg"] = "定位状态接口处理失败: " + std::string(e.what());
                response["successed"] = false;
                res.result(http::status::internal_server_error);
                boost::beast::ostream(res.body()) << response.dump(4);
                res.prepare_payload();
            }
            return;
        }
        // 其他请求
        else {
            res.set(http::field::content_type, "application/json");
            res.result(http::status::not_found);
            json response_json = {{"error", "无效的请求路径或方法"}};
            beast::ostream(res.body()) << response_json.dump();
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "请求处理错误: %s", e.what());
        res.set(http::field::content_type, "application/json");
        res.result(http::status::bad_request);
        json response_json = {{"error", e.what()}};
        beast::ostream(res.body()) << response_json.dump();
    }
    
    res.prepare_payload();
}
