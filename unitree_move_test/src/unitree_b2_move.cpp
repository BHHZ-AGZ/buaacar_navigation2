#include <iostream>
#include <string>
#include <chrono>
#include <mutex>
#include <thread>
#include <pthread.h>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace B2W {
    class B2WInterface;
}

namespace B2W {
    extern B2WInterface* B2WInstance();
    extern void B2WInstanceDestroy(B2WInterface* instance);
    extern bool Init(B2WInterface* obj,std::string network);
    extern int32_t EmergencyStop(B2WInterface* obj);
    extern int32_t StopMove(B2WInterface* obj);
    extern int32_t StandUp(B2WInterface* obj);
    extern int32_t StandDown(B2WInterface* obj);
    extern int32_t RecoveryStand(B2WInterface* obj);
    extern int32_t Move(B2WInterface* obj,float vx, float vy, float vyaw);
    extern int32_t SwitchGait(B2WInterface* obj,int d);
    extern bool StartMapping(B2WInterface* obj);
    extern bool EndMapping(B2WInterface* obj);
    extern bool AddNode(B2WInterface* obj,uint16_t nodename,std::vector<double> pose);
    extern bool DeleteNode(B2WInterface* obj,uint16_t nodename=999);
    extern bool AddEdge(B2WInterface* obj,uint16_t edgename,uint16_t start_node,uint16_t end_node,float speed=1);
    extern bool DeleteEdge(B2WInterface* obj,uint16_t edgename=999);
    extern bool StartNav(B2WInterface* obj);
    extern bool PauseNav(B2WInterface* obj);
    extern bool RecoverNav(B2WInterface* obj);
    extern bool SingleNav(B2WInterface* obj,uint16_t nodename);
    extern bool ReturnNav(B2WInterface* obj);
    extern bool StartRelocation(B2WInterface* obj);
    extern bool RelocationInit(B2WInterface* obj);
    extern bool CloseNav(B2WInterface* obj);
    extern std::vector<double> GetB2WPosition(B2WInterface* obj);
    extern double GetDistance(B2WInterface* obj);
    extern std::vector<double> GetDistFrontLeftRight(B2WInterface* obj);
    extern std::vector<double> GetDistAngle(B2WInterface* obj);

}

class B2WMoveNode : public rclcpp::Node
{
public:
    B2WMoveNode(const std::string& network_interface)
        : Node("b2w_move_node"),
          b2w_(nullptr),
          current_vx_(0.0f),
          current_vy_(0.0f),
          current_vyaw_(0.0f)
    {
        b2w_ = B2W::B2WInstance();
        if (!b2w_) {
            throw std::runtime_error("Failed to create B2WInstance!");
        }
        if (!B2W::Init(b2w_, network_interface)) {
            throw std::runtime_error("Failed to init B2WInterface with network: " + network_interface);
        }

        
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 
            10,                 
            std::bind(&B2WMoveNode::cmd_vel_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "B2WMoveNode initialized successfully!");
        
    }

    ~B2WMoveNode() override
    {
        if (b2w_) {
            B2W::StopMove(b2w_);
            B2W::StandDown(b2w_);
            B2W::B2WInstanceDestroy(b2w_);
            RCLCPP_INFO(this->get_logger(), "B2WInstance destroyed.");
        }
        RCLCPP_INFO(this->get_logger(), "B2WMoveNode closed.");
    }

    bool stand_up()
    {
        int32_t res = B2W::StandUp(b2w_);
        if (res != 0) {
            RCLCPP_ERROR(this->get_logger(), "StandUp failed! res: %d", res);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Robot stand up success.");
        return true;
    }

    void run_move_loop()
    {
        RCLCPP_INFO(this->get_logger(), "运动循环已启动(频率20Hz)");
        rclcpp::Rate rate(20);

        while (rclcpp::ok()) {
            // 锁仅保护参数拷贝，作用域最小化
            float vx = 0.0f, vy = 0.0f, vyaw = 0.0f;
            {   // 缩小锁的作用域（仅拷贝参数时加锁）
                std::lock_guard<std::mutex> lock(vel_mutex_);
                vx = current_vx_;
                vy = current_vy_;
                vyaw = current_vyaw_;
            }

            int32_t res = B2W::Move(b2w_, vx, vy, vyaw);
            RCLCPP_INFO(this->get_logger(), "res= %d | 当前速度: vx=%.2f, vy=%.2f, vyaw=%.2f", 
                        res, vx, vy, vyaw);

            if (res != 0) {
                RCLCPP_WARN(this->get_logger(), "Move failed! res: %d | vx: %.2f, vy: %.2f, vyaw: %.2f",
                            res, vx, vy, vyaw);
            } else {
                RCLCPP_INFO(this->get_logger(), "Move cmd sent: vx=%.2f, vy=%.2f, vyaw=%.2f",
                            vx, vy, vyaw); 
            }

          
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(vel_mutex_);
        
        current_vx_ = static_cast<float>(msg->linear.x);
        current_vy_ = static_cast<float>(msg->linear.y);
        current_vyaw_ = static_cast<float>(msg->angular.z);

        RCLCPP_INFO(this->get_logger(), "Received /turtle1/cmd_vel: vx=%.2f, vy=%.2f, vyaw=%.2f",
                     current_vx_, current_vy_, current_vyaw_);
    }

    B2W::B2WInterface* b2w_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    float current_vx_;
    float current_vy_;
    float current_vyaw_;
    std::mutex vel_mutex_;
};

int main(int argc, char **argv)
{
    const rclcpp::Logger logger = rclcpp::get_logger("b2w_move_main");

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <networkInterface>" << std::endl;
        std::cerr << "Example: " << argv[0] << " eth0" << std::endl;
        return -1;
    }
    std::string network_interface = argv[1];

    //setenv("FASTRTPS_DEFAULT_PROFILES_FILE", "", 1);
    setenv("ROS_DISABLE_SHM", "1", 1);

    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<B2WMoveNode>(network_interface);
        if (node->stand_up()) {
            std::this_thread::sleep_for(std::chrono::seconds(1)); // 缩短休眠，快速进入循环
            node->run_move_loop();
        } else {
            RCLCPP_FATAL(logger, "机器人起立失败，无法进入运动循环！");
            rclcpp::shutdown();
            return -1;
        }
    } catch (const std::exception& e) {
        RCLCPP_FATAL(logger, "Error: %s", e.what());
        rclcpp::shutdown();
        return -1;
    }

    rclcpp::shutdown();
    return 0;
}