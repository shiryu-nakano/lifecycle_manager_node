#include "local_planning_manager/local_planning_manager_node.hpp"

static constexpr const char *PURE_PURSUIT_NODE = "pure_pursuit_node";
static constexpr const char *DWA_NODE = "dwa_node";
static constexpr const char *STOP_MOTION_NODE = "stop_motion_node"; // 修正
static constexpr const char *IN_PLACE_TURN_NODE = "in_place_turn_node";

namespace local_planning_manager
{

    // コンストラクタ
    LocalPlanningManagerNode::LocalPlanningManagerNode(const rclcpp::NodeOptions &options)
        : Node("local_planning_manager_node", options)
    {

        // パラメータの宣言と取得
        const double tick_hz = this->declare_parameter<double>("tick_hz", 2.0); // [Hz]

        // Component初期化
        component_ = std::make_unique<LocalPlanningManagerComponent>();

        // 初期値
        // TODO 立ち上げる時の値をコンストラクタで与えられるようにする．もちろんdefault値も設定する
        current_state_ = "PurePursuit";
        obstacle_detected_ = false;
        obstacle_distance_ = std::numeric_limits<double>::infinity();
        current_x_ = 0.0;
        current_y_ = 0.0;
        state_entered_at_ = std::chrono::steady_clock::now();


        // --- Subscribers ---
        using std::placeholders::_1;
        sub_current_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/gnss_pose", 10, std::bind(&LocalPlanningManagerNode::currentPoseCallback, this, _1));
        sub_obstacle_detected_ = this->create_subscription<std_msgs::msg::Bool>(
            "/obstacle_detected", 10, std::bind(&LocalPlanningManagerNode::obstacleDetectedCallback, this, _1));
        sub_obstacle_distance_ = this->create_subscription<std_msgs::msg::Float64>(
            "/obstacle_distance", 10, std::bind(&LocalPlanningManagerNode::obstacleDistanceCallback, this, _1));

        // Timer (10Hz)
        // --- Timer ---
        auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, tick_hz)); // 周期[s]
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&LocalPlanningManagerNode::timerCallback, this));

        // initialize others
        createClients();
        createTransitionMap();

        RCLCPP_INFO(this->get_logger(), "LocalPlanningManagerNode initialized");
    }



    // ===== Callbacks =====
    // Pose callback
    void LocalPlanningManagerNode::currentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_x_ = msg->pose.position.x;
        current_y_ = msg->pose.position.y;
    }
    // Obstacle detected
    void LocalPlanningManagerNode::obstacleDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        obstacle_detected_ = msg->data;
    }
    // Obstacle distance
    void LocalPlanningManagerNode::obstacleDistanceCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // TODO OK??
        obstacle_distance_ = msg->data;
    }
    // Timer Callback
    void LocalPlanningManagerNode::timerCallback()
    {
        // TransitionContextの構築
        TransitionContext context;
        context.current_state = current_state_;
        context.obstacle_detected = obstacle_detected_;
        context.obstacle_distance = obstacle_distance_;
        context.elapsed_time = getElapsedTime();
        context.current_x = current_x_;
        context.current_y = current_y_;
        
        // Componentに遷移判定を依頼
        auto recipe_opt = component_->decideTransition(context);
        
        // Recipeが返された場合は実行
        if (recipe_opt.has_value())
        {
            executeTransitionRecipe(recipe_opt.value());
            
            // 状態遷移が発生したので時刻を更新
            state_entered_at_ = std::chrono::steady_clock::now();
            std::optional<std::string> new_state = getCurrentState();
            // TODO 危険かも
            RCLCPP_INFO(this->get_logger(), "State transitioned from %s to %s", current_state_.c_str(), new_state.value_or(current_state_).c_str());
            current_state_ = new_state.value_or(current_state_);
        }
    }





    // ===== Helpers =====
    // Transition execution
    void LocalPlanningManagerNode::executeTransitionRecipe(const TransitionRecipe& recipe)
    {
        RCLCPP_INFO(this->get_logger(), "Executing recipe: %s", recipe.description.c_str());

        for (const ActionStep &step : recipe.steps)
        {
            // --- (1) クライアントの取得 ---
            rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client;

            try
            {
                client = lifecycle_clients_.at(step.target_node_name);
            }
            catch (const std::out_of_range &)
            {
                RCLCPP_WARN(this->get_logger(), "Unknown target node '%s'; skipping step.",
                            step.target_node_name.c_str());
                continue;
            }

            // --- (2) サービスの待機
            const double tout_s = (step.timeout_s > 0.0) ? step.timeout_s : 5.0;
            if (!client->wait_for_service(std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::duration<double>(tout_s))))
            {
                RCLCPP_WARN(this->get_logger(), "%s/change_state not available within %.1fs",
                            step.target_node_name.c_str(), tout_s);
                continue;
            }

            // --- (3) Transition ID の取得 ---
            uint8_t transition_id;

            try
            {
                transition_id = transition_map_.at(step.operation);
            }
            catch (const std::out_of_range &)
            {
                RCLCPP_WARN(this->get_logger(), "Unknown operation '%s'; skipping step.",
                            step.operation.c_str());
                continue;
            }

            // --- (4) リクエストの作成と送信 ---
            auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
            req->transition.id = transition_id;

            auto fut = client->async_send_request(req);
            auto rc = rclcpp::spin_until_future_complete(
                this->get_node_base_interface(), fut,
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(tout_s)));

            // --- (5) 結果の判定 ---
            if (rc != rclcpp::FutureReturnCode::SUCCESS || !fut.get()->success)
            {
                RCLCPP_WARN(this->get_logger(), "Transition '%s' for %s failed or timed out",
                            step.operation.c_str(), step.target_node_name.c_str());
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Transition '%s' for %s succeeded",
                            step.operation.c_str(), step.target_node_name.c_str());
            }
        }
    }

    // 状態遷移してからの時間計測
    double LocalPlanningManagerNode::getElapsedTime() const
    {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - state_entered_at_);
        return elapsed.count();
    }

    // 現在のセマンティック状態取得
    // TimerCallbackよりも低頻度で実行することにする．
    std::optional<std::string> LocalPlanningManagerNode::getCurrentState()
    {
        // まず管理対象のノード群の状態を取得する
        SemanticState semantic_state;
        
        for (const auto &pair : lifecycle_get_clients_)
        {
            const std::string &node_name = pair.first;
            auto client = pair.second;
            
            // サービスが利用可能か確認
            if (!client->wait_for_service(std::chrono::milliseconds(500)))
            {
                RCLCPP_WARN(this->get_logger(), "GetState service not available for %s", node_name.c_str());
                semantic_state.semantic_state[node_name] = SemanticState::State::UNKNOWN;
                continue;
            }
            
            // リクエスト作成と送信
            auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
            auto future = client->async_send_request(request);
            
            // 応答待機
            auto status = rclcpp::spin_until_future_complete(
                this->get_node_base_interface(),
                future,
                std::chrono::seconds(1));
            
            if (status == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = future.get();
                // ROS2 lifecycle state IDをSemanticState::Stateにマッピング
                switch (response->current_state.id)
                {
                    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
                        semantic_state.semantic_state[node_name] = SemanticState::State::UNCONFIGURED;
                        break;
                    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
                        semantic_state.semantic_state[node_name] = SemanticState::State::INACTIVE;
                        break;
                    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
                        semantic_state.semantic_state[node_name] = SemanticState::State::ACTIVE;
                        break;
                    case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:
                        semantic_state.semantic_state[node_name] = SemanticState::State::FINALIZED;
                        break;
                    default:
                        semantic_state.semantic_state[node_name] = SemanticState::State::UNKNOWN;
                        break;
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Failed to get state for %s", node_name.c_str());
                semantic_state.semantic_state[node_name] = SemanticState::State::UNKNOWN;
            }
        }
        
        return component_->getCurrentState(semantic_state);
    }



    
    // ===== Initialization Helpers=====
    // ChangeState/ GetStateクライアントの作成
    // TODO 引数にnode群を渡して作成できるようにすること
    void LocalPlanningManagerNode::createClients()
    {
        // ライフサイクルクライアントを作成するラムダ関数
        auto create_change_client = [&](const std::string &node_name)
        {
            return this->create_client<lifecycle_msgs::srv::ChangeState>(
                node_name + "/change_state");
        };
        auto create_get_client = [&](const std::string &node_name)
        {
            return this->create_client<lifecycle_msgs::srv::GetState>(
                node_name + "/get_state");
        };

        for (const auto &node_name : {PURE_PURSUIT_NODE, DWA_NODE, STOP_MOTION_NODE, IN_PLACE_TURN_NODE})
        {
            lifecycle_clients_[node_name] = create_change_client(std::string("/") + node_name);
            lifecycle_get_clients_[node_name] = create_get_client(std::string("/") + node_name);
        }
        RCLCPP_INFO(this->get_logger(), "Lifecycle clients created");
    }





    void LocalPlanningManagerNode::createTransitionMap()
    {
        // --- トランジションマップの初期化 ---
        transition_map_["configure"]  = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
        transition_map_["activate"]   = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
        transition_map_["deactivate"] = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
        transition_map_["cleanup"]    = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;
        transition_map_["shutdown"]   = lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN;

        RCLCPP_INFO(this->get_logger(), "Transition map initialized");
    }


} // namespace local_planning_manager
