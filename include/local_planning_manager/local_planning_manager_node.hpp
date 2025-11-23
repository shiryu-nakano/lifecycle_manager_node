#pragma once
#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "local_planning_manager_component.hpp"
#include "common_types.hpp"
#include <memory>
#include <map>
#include <string>
#include <chrono>

namespace local_planning_manager
{

/**
 * @brief ROSインターフェースを担当するNodeクラス
 * 
 * Strategy/Handler/CoRパターンの実装詳細を一切知らない。
 * Componentのパブリックインターフェースのみを使用。
 */
class LocalPlanningManagerNode : public rclcpp::Node {
public:
    explicit LocalPlanningManagerNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~LocalPlanningManagerNode() = default;

private:
    // Component
    std::unique_ptr<LocalPlanningManagerComponent> component_;
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_obstacle_detected_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_obstacle_distance_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_current_pose_;

    // Lifecycle clients
    std::map<std::string, // lifecycle node name
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> 
        lifecycle_clients_;
    std::map<std::string,
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr>
        lifecycle_get_clients_;
    
    std::map<std::string, uint8_t> transition_map_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // 現在の状態になってからの経過時間計測
    std::chrono::steady_clock::time_point state_entered_at_;
    
    // State tracking
    std::string current_state_;
    bool obstacle_detected_;
    double obstacle_distance_;
    double current_x_;
    double current_y_;
    

    // ===== Callbacks =====
    void timerCallback();
    void obstacleDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void obstacleDistanceCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void currentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // ===== Helpers =====
    // Transition execution
    void executeTransitionRecipe(const TransitionRecipe& recipe);
    // 状態遷移してからの時間計測
    double getElapsedTime() const;
    // 現在のセマンティック状態取得
    std::optional<std::string> getCurrentState();
    
    // ===== Initialization Helpers=====
    void createClients();
    void createTransitionMap();
};

} // namespace local_planning_manager