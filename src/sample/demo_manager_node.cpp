#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "transition_recipe_test/local_planning_manager_component.hpp"
#include "transition_recipe_test/sample/sample_strategies.hpp"

#include <chrono>
#include <map>
#include <memory>
#include <string>

using namespace std::chrono_literals;

/**
 * @brief デモ用マネージャノード
 *
 * 「LocalPlanningManagerComponent をどう使うか」を示すリファレンス実装。
 * - 共通型 (TransitionContext / TransitionRecipe / ActionStep) をそのまま使用
 * - サンプルの戦略 (SampleAreaA/B/CStrategy) を Component に登録
 * - 現在位置 (x) に応じてエリア A→B→C と切り替わり、対応するライフサイクルノードを activate
 *
 * オリジナルのプロジェクトに適用する際は、独自の戦略・ハンドラを作成し
 * registerAreaStrategy / registerAreaSwitchCondition で差し替えればよい。
 */
class DemoManagerNode : public rclcpp::Node
{
public:
    DemoManagerNode() : Node("demo_manager_node")
    {
        using namespace transition_recipe_test;
        using namespace transition_recipe_test::sample;

        // パラメータ
        const double tick_hz = this->declare_parameter<double>("tick_hz", 2.0);
        const double area_a_threshold = this->declare_parameter<double>("area_a_threshold", 5.0);
        const double area_b_threshold = this->declare_parameter<double>("area_b_threshold", 10.0);

        // ===== Component 構築（メインの実装をそのまま使う） =====
        component_ = std::make_unique<LocalPlanningManagerComponent>();
        component_->setAreaList({"A", "B", "C"});
        // サンプルではエリア切替を「x が閾値を越えたか」で判断する。
        // (target_y, threshold) は y は無視できるよう threshold を大きめに取る単純実装。
        component_->registerAreaSwitchCondition("A", area_a_threshold, 0.0, 0.5);
        component_->registerAreaSwitchCondition("B", area_b_threshold, 0.0, 0.5);
        component_->registerAreaStrategy("A", std::make_unique<SampleAreaAStrategy>());
        component_->registerAreaStrategy("B", std::make_unique<SampleAreaBStrategy>());
        component_->registerAreaStrategy("C", std::make_unique<SampleAreaCStrategy>());
        component_->initialize();

        // ===== 初期状態 =====
        current_state_ = SampleStateID::STRAIGHT_DRIVE;
        current_x_ = 0.0;
        current_y_ = 0.0;
        state_entered_at_ = std::chrono::steady_clock::now();

        // ===== Subscriber / Publisher =====
        sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "current_pose", 10,
            std::bind(&DemoManagerNode::poseCallback, this, std::placeholders::_1));
        pub_state_ = this->create_publisher<std_msgs::msg::String>("current_state", 10);

        // ===== Lifecycle clients =====
        createClients();
        createTransitionMap();

        // ===== Timer =====
        const auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, tick_hz));
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&DemoManagerNode::timerCallback, this));

        // ===== ライフサイクルノードの初期化（configure → 初期 activate） =====
        bootstrap_timer_ = this->create_wall_timer(
            1s, std::bind(&DemoManagerNode::bootstrapOnce, this));

        RCLCPP_INFO(this->get_logger(),
                    "DemoManagerNode initialized (using LocalPlanningManagerComponent)");
    }

private:
    using TransitionContext = transition_recipe_test::TransitionContext;
    using TransitionRecipe = transition_recipe_test::TransitionRecipe;
    using ActionStep = transition_recipe_test::ActionStep;

    // ===== Callbacks =====
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_x_ = msg->pose.position.x;
        current_y_ = msg->pose.position.y;
    }

    void timerCallback()
    {
        TransitionContext context;
        context.current_state = current_state_;
        context.obstacle_detected = false;
        context.obstacle_distance = std::numeric_limits<double>::infinity();
        context.elapsed_time = getElapsedTime();
        context.current_x = current_x_;
        context.current_y = current_y_;

        auto recipe_opt = component_->decideTransition(context);
        if (recipe_opt.has_value())
        {
            executeTransitionRecipe(recipe_opt.value());
            state_entered_at_ = std::chrono::steady_clock::now();
            // 遷移後の状態を推定：レシピ内の最後の activate に対応する状態名へ
            for (const auto &step : recipe_opt.value().steps)
            {
                if (step.operation == "activate")
                {
                    auto state = nodeNameToState(step.target_node_name);
                    if (state)
                    {
                        RCLCPP_INFO(this->get_logger(),
                                    "State transitioned: %s -> %s",
                                    current_state_.c_str(), state->c_str());
                        current_state_ = *state;
                    }
                }
            }
        }

        std_msgs::msg::String state_msg;
        state_msg.data = current_state_;
        pub_state_->publish(state_msg);
    }

    // ===== Recipe 実行（非同期） =====
    void executeTransitionRecipe(const TransitionRecipe &recipe)
    {
        RCLCPP_INFO(this->get_logger(), "Executing recipe: %s", recipe.description.c_str());
        for (const auto &step : recipe.steps)
        {
            auto cli_it = lifecycle_clients_.find(step.target_node_name);
            if (cli_it == lifecycle_clients_.end())
            {
                RCLCPP_WARN(this->get_logger(), "Unknown target node '%s'",
                            step.target_node_name.c_str());
                continue;
            }
            auto trans_it = transition_map_.find(step.operation);
            if (trans_it == transition_map_.end())
            {
                RCLCPP_WARN(this->get_logger(), "Unknown operation '%s'", step.operation.c_str());
                continue;
            }
            if (!cli_it->second->service_is_ready())
            {
                RCLCPP_WARN(this->get_logger(), "%s/change_state not ready; skipping",
                            step.target_node_name.c_str());
                continue;
            }
            sendChangeStateAsync(step.target_node_name, trans_it->second, step.operation);
        }
    }

    // ===== ライフサイクル初期化（一度だけ、非同期） =====
    void bootstrapOnce()
    {
        if (bootstrapped_)
        {
            return;
        }
        bootstrapped_ = true;
        bootstrap_timer_->cancel();

        RCLCPP_INFO(this->get_logger(), "Bootstrapping sample lifecycle nodes...");
        for (const auto &node_name : {"straight_drive_node", "slow_drive_node", "stop_node"})
        {
            sendChangeStateAsync(node_name,
                                 lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
                                 "configure");
        }
        // 初期状態は StraightDrive。configure の完了を多少待ってから activate を投げる。
        activate_timer_ = this->create_wall_timer(
            500ms, [this]() {
                activate_timer_->cancel();
                sendChangeStateAsync("straight_drive_node",
                                     lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
                                     "activate");
                RCLCPP_INFO(this->get_logger(), "Bootstrap complete");
            });
    }

    /// ChangeState サービスを非同期で呼び、結果をログに出すだけのヘルパ。
    /// timer コールバック内から呼んでも executor 再入は発生しない。
    void sendChangeStateAsync(const std::string &node_name, uint8_t transition_id,
                              const std::string &operation_label)
    {
        auto cli_it = lifecycle_clients_.find(node_name);
        if (cli_it == lifecycle_clients_.end())
        {
            return;
        }
        auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        req->transition.id = transition_id;
        const std::string node_name_copy = node_name;
        const std::string op_copy = operation_label;
        cli_it->second->async_send_request(
            req,
            [this, node_name_copy, op_copy](
                rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future) {
                auto result = future.get();
                if (result && result->success)
                {
                    RCLCPP_INFO(this->get_logger(), "Transition '%s' for %s succeeded",
                                op_copy.c_str(), node_name_copy.c_str());
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Transition '%s' for %s failed",
                                op_copy.c_str(), node_name_copy.c_str());
                }
            });
    }

    // ===== Helpers =====
    double getElapsedTime() const
    {
        const auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - state_entered_at_).count();
    }

    std::optional<std::string> nodeNameToState(const std::string &node_name) const
    {
        using namespace transition_recipe_test::sample;
        if (node_name == "straight_drive_node") return SampleStateID::STRAIGHT_DRIVE;
        if (node_name == "slow_drive_node")     return SampleStateID::SLOW_DRIVE;
        if (node_name == "stop_node")           return SampleStateID::STOP;
        return std::nullopt;
    }

    void createClients()
    {
        for (const auto &node_name : {"straight_drive_node", "slow_drive_node", "stop_node"})
        {
            lifecycle_clients_[node_name] =
                this->create_client<lifecycle_msgs::srv::ChangeState>(
                    std::string("/") + node_name + "/change_state");
        }
        RCLCPP_INFO(this->get_logger(), "Lifecycle clients created");
    }

    void createTransitionMap()
    {
        transition_map_["configure"] = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
        transition_map_["activate"] = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
        transition_map_["deactivate"] = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
        transition_map_["cleanup"] = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;
    }

    // ===== State =====
    std::unique_ptr<transition_recipe_test::LocalPlanningManagerComponent> component_;
    std::string current_state_;
    double current_x_;
    double current_y_;
    std::chrono::steady_clock::time_point state_entered_at_;

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
    std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> lifecycle_clients_;
    std::map<std::string, uint8_t> transition_map_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr bootstrap_timer_;
    rclcpp::TimerBase::SharedPtr activate_timer_;
    bool bootstrapped_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DemoManagerNode>());
    rclcpp::shutdown();
    return 0;
}
