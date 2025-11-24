#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "local_planning_manager/common_types.hpp"
#include "local_planning_manager/demo_manager_component.hpp"

using namespace std::chrono_literals;

/**
 * @brief デモ用のマネージャーノード
 *
 * ロボットの位置に基づいてエリアを判定し、
 * 適切な戦略を選択してライフサイクルノードを管理
 */
class DemoManagerNode : public rclcpp::Node
{
public:
    DemoManagerNode() : Node("demo_manager_node")
    {
        // パラメータ
        this->declare_parameter<double>("tick_hz", 2.0);
        this->declare_parameter<double>("area_a_threshold", 5.0);
        this->declare_parameter<double>("area_b_threshold", 5.0);

        double tick_hz = this->get_parameter("tick_hz").as_double();
        area_a_threshold_ = this->get_parameter("area_a_threshold").as_double();
        area_b_threshold_ = this->get_parameter("area_b_threshold").as_double();

        // Component初期化
        component_ = std::make_unique<demo::DemoManagerComponent>();

        // 初期化
        current_state_ = demo::StateID::STRAIGHT_DRIVE;
        current_x_ = 0.0;
        current_y_ = 0.0;
        state_entered_at_ = std::chrono::steady_clock::now();

        // Subscriber
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "current_pose", 10,
            std::bind(&DemoManagerNode::pose_callback, this, std::placeholders::_1));

        // Publishers
        area_pub_ = this->create_publisher<std_msgs::msg::String>("current_area", 10);
        state_pub_ = this->create_publisher<std_msgs::msg::String>("current_state", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("area_markers", 10);

        // Lifecycle clients
        create_lifecycle_clients();
        create_transition_map();

        // Timer
        auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, tick_hz));
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&DemoManagerNode::timer_callback, this));

        // 初期状態を設定（straight_drive_nodeをactivate）
        initialize_nodes();

        // エリアマーカーを定期的にパブリッシュ
        marker_timer_ = this->create_wall_timer(
            1s, std::bind(&DemoManagerNode::publish_area_markers, this));

        RCLCPP_INFO(this->get_logger(), "DemoManagerNode initialized");
    }

private:
    void create_lifecycle_clients()
    {
        // でもで使うnode群のライフサイクルクライアントを作成
        const std::vector<std::string> node_names = {
            "straight_drive_node",
            "slow_drive_node",
            "stop_node"};

        for (const auto &node_name : node_names)
        {
            std::string service_name = "/" + node_name + "/change_state";
            lifecycle_clients_[node_name] =
                this->create_client<lifecycle_msgs::srv::ChangeState>(service_name);

            std::string get_service_name = "/" + node_name + "/get_state";
            lifecycle_get_clients_[node_name] =
                this->create_client<lifecycle_msgs::srv::GetState>(get_service_name);
        }
    }

    void create_transition_map()
    {
        transition_map_["configure"] = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
        transition_map_["activate"] = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
        transition_map_["deactivate"] = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
        transition_map_["cleanup"] = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;
    }

    void initialize_nodes()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing lifecycle nodes...");

        // 全ノードをconfigureする
        std::this_thread::sleep_for(1s); // ノードの起動を待つ

        configure_node("straight_drive_node");
        configure_node("slow_drive_node");
        configure_node("stop_node");

        // straight_drive_nodeのみactivateする
        std::this_thread::sleep_for(500ms);
        activate_node("straight_drive_node");

        RCLCPP_INFO(this->get_logger(), "Initialization complete");
    }

    void configure_node(const std::string &node_name)
    {
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;

        auto client = lifecycle_clients_[node_name];
        if (!client->wait_for_service(5s))
        {
            RCLCPP_WARN(this->get_logger(), "Service %s not available", node_name.c_str());
            return;
        }

        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result, 5s) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Configured %s", node_name.c_str());
        }
    }

    void activate_node(const std::string &node_name)
    {
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;

        auto client = lifecycle_clients_[node_name];
        if (!client->wait_for_service(5s))
        {
            RCLCPP_WARN(this->get_logger(), "Service %s not available", node_name.c_str());
            return;
        }

        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result, 5s) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Activated %s", node_name.c_str());
        }
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_x_ = msg->pose.position.x;
        current_y_ = msg->pose.position.y;
    }

    void timer_callback()
    {
        // 遷移コンテキストの構築
        demo::DemoTransitionContext context;
        context.current_state = current_state_;
        context.current_x = current_x_;
        context.current_y = current_y_;
        context.elapsed_time = get_elapsed_time();

        // Componentに遷移判定を依頼
        auto recipe_opt = component_->decideTransition(context);

        // Recipeが返された場合は実行
        if (recipe_opt.has_value())
        {
            RCLCPP_INFO(this->get_logger(), "Transition recipe available: %s",
                        recipe_opt.value().description.c_str());
            execute_transition_recipe(recipe_opt.value());
            state_entered_at_ = std::chrono::steady_clock::now();

            // 状態の更新
            update_current_state();
        }

        // 現在のエリアを取得
        auto current_area_opt = component_->getCurrentArea();
        if (current_area_opt.has_value())
        {
            current_area_ = current_area_opt.value();
        }

        // パブリッシュ
        publish_current_area();
        publish_current_state();
    }

    void update_current_state()
    {
        // 遷移後の状態を更新
        std::string new_state;
        if (current_x_ < area_a_threshold_)
        {
            new_state = demo::StateID::STRAIGHT_DRIVE;
        }
        else if (current_x_ < area_b_threshold_)
        {
            new_state = demo::StateID::SLOW_DRIVE;
        }
        else
        {
            new_state = demo::StateID::STOP;
        }

        if (new_state != current_state_)
        {
            RCLCPP_INFO(this->get_logger(), "State updated: %s -> %s",
                        current_state_.c_str(), new_state.c_str());
            current_state_ = new_state;
        }
    }

    void execute_transition_recipe(const demo::DemoTransitionRecipe &recipe)
    {
        RCLCPP_INFO(this->get_logger(), "Executing transition: %s", recipe.description.c_str());

        for (const auto &step : recipe.steps)
        {
            auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
            request->transition.id = transition_map_[step.operation];

            auto client = lifecycle_clients_[step.target_node_name];

            if (!client->wait_for_service(std::chrono::duration<double>(step.timeout_s)))
            {
                RCLCPP_WARN(this->get_logger(), "Service %s not available",
                            step.target_node_name.c_str());
                continue;
            }

            auto result = client->async_send_request(request);
            auto timeout = std::chrono::duration<double>(step.timeout_s);

            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result, timeout) ==
                rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_INFO(this->get_logger(), "Successfully executed: %s on %s",
                            step.operation.c_str(), step.target_node_name.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to execute: %s on %s",
                             step.operation.c_str(), step.target_node_name.c_str());
            }
        }
    }

    double get_elapsed_time() const
    {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - state_entered_at_).count();
    }

    void publish_current_area()
    {
        std_msgs::msg::String msg;
        msg.data = current_area_;
        area_pub_->publish(msg);
    }

    void publish_current_state()
    {
        std_msgs::msg::String msg;
        msg.data = current_state_;
        state_pub_->publish(msg);
    }

    void publish_area_markers()
    {
        visualization_msgs::msg::MarkerArray marker_array;

        // エリア境界線を追加
        marker_array.markers.push_back(create_area_boundary_marker(0, area_a_threshold_, 1.0, 0.0, 0.0));
        marker_array.markers.push_back(create_area_boundary_marker(1, area_b_threshold_, 0.0, 0.0, 1.0));

        // エリアテキストマーカーを追加
        marker_array.markers.push_back(create_area_text_marker(2, area_a_threshold_ / 2.0, "Area A"));
        marker_array.markers.push_back(create_area_text_marker(3, (area_a_threshold_ + area_b_threshold_) / 2.0, "Area B"));
        marker_array.markers.push_back(create_area_text_marker(4, area_b_threshold_ + 2.0, "Area C"));

        marker_pub_->publish(marker_array);
    }

    visualization_msgs::msg::Marker create_area_boundary_marker(int id, double x, double r, double g, double b)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = this->now();
        marker.ns = "area_boundaries";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point p1, p2;
        p1.x = x;
        p1.y = -5.0;
        p1.z = 0.0;
        p2.x = x;
        p2.y = 5.0;
        p2.z = 0.0;

        marker.points.push_back(p1);
        marker.points.push_back(p2);

        marker.scale.x = 0.05;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;

        return marker;
    }

    visualization_msgs::msg::Marker create_area_text_marker(int id, double x, const std::string &text)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = this->now();
        marker.ns = "area_labels";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = x;
        marker.pose.position.y = 3.0;
        marker.pose.position.z = 0.5;
        marker.pose.orientation.w = 1.0;

        marker.scale.z = 0.5;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        marker.text = text;

        return marker;
    }

    // State variables
    std::unique_ptr<demo::DemoManagerComponent> component_;
    std::string current_state_;
    std::string current_area_;
    double current_x_, current_y_;
    std::chrono::steady_clock::time_point state_entered_at_;

    double area_a_threshold_;
    double area_b_threshold_;

    // Lifecycle clients
    std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> lifecycle_clients_;
    std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> lifecycle_get_clients_;
    std::map<std::string, uint8_t> transition_map_;

    // ROS interface
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr area_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr marker_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DemoManagerNode>());
    rclcpp::shutdown();
    return 0;
}
