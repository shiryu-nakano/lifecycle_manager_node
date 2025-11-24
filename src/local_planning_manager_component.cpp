#include "local_planning_manager/local_planning_manager_component.hpp"
#include "local_planning_manager/core/strategy.hpp"
#include "core/strategies/area_a_strategy.cpp"
#include "core/strategies/area_b_strategy.cpp"
#include "core/strategies/area_c_strategy.cpp"
#include <limits>
#include <iostream>
#include <cmath>

namespace local_planning_manager
{

    // コンストラクタ
    // TODO
    LocalPlanningManagerComponent::LocalPlanningManagerComponent()
        : current_area_index_(0), current_area_strategy_(nullptr)
    {
        // デフォルト設定（YAML読み込み前の仮設定）
        setAreaList({"A", "B", "C"});

        // デフォルトの切り替え条件
        registerAreaSwitchCondition("A", 10.0, 5.0, 2.0);  // A to B
        registerAreaSwitchCondition("B", 20.0, 10.0, 2.0); // B to C

        // 全てのStrategyを事前に作成してmapに登録
        // initializeStrategies();

        area_strategies_["A"] = std::make_unique<AreaAStrategy>();
        area_strategies_["B"] = std::make_unique<AreaBStrategy>();
        area_strategies_["C"] = std::make_unique<AreaCStrategy>();

        // 初期エリアの戦略を設定
        if (!area_list_.empty())
        {
            current_area_id_ = area_list_[current_area_index_];
            setAreaStrategy(current_area_id_);
        }
    }

    // Private Initialize Helper
    // Decrare Strategies (currently unused)
    void LocalPlanningManagerComponent::initializeStrategies()
    {
        // すべてのエリア戦略を事前に作成
        return;
        // 将来的にはFactory + YAMLで動的生成
        // for (const auto& area_id : yaml_config["areas"]) {
        //     area_strategies_[area_id] = StrategyFactory::create(area_id);
        // }
    }

    // Initialize Area ID LIST
    void LocalPlanningManagerComponent::setAreaList(const std::vector<std::string> &area_list)
    {
        area_list_ = area_list;
        current_area_index_ = 0;
        if (!area_list_.empty())
        {
            current_area_id_ = area_list_[0];
        }
    }

    // Set Switch point and threshold of each 
    void LocalPlanningManagerComponent::registerAreaSwitchCondition(const std::string &area_id, double target_x, double target_y, double threshold)
    {
        area_switch_conditions_[area_id] = std::make_tuple(target_x, target_y, threshold);
    }

    // ===== public  methods =====
    // Confirm current SemanticState
    std::optional<std::string> LocalPlanningManagerComponent::getCurrentState(SemanticState semantic) const
    {
        return state_graph_.getCurrentSemanticState(semantic);
    }

    // Decide whether or not Transit to other Mode
    std::optional<TransitionRecipe> LocalPlanningManagerComponent::decideTransition(const TransitionContext &context)
    {

        // 最終エリアに到達していない時，
        if (current_area_index_ < static_cast<int>(area_list_.size()) - 1)
        {
            if (shouldSwitchArea(context.current_x, context.current_y))
            {
                // update index and id
                std::string pre_area_id = area_list_[current_area_index_];
                current_area_index_++;
                current_area_id_ = area_list_[current_area_index_];
                
                // エリア戦略の切り替え
                setAreaStrategy(current_area_id_);

                std::cout << "Area switched from " << pre_area_id << " to: " << current_area_id_ << std::endl;
            }
        }

        // 現在のエリア戦略で遷移判定
        if (current_area_strategy_ != nullptr)
        {
            return current_area_strategy_->processTransitions(context);
        }

        return std::nullopt;

        // std::string current_area_id = getCurrentArea(x, y);
    }
    
    // Get Current Area ID
    /*
    std::string LocalPlanningManagerComponent::getCurrentArea(double x, double y)
    {
        return current_area_id_;
    }
    */
    // ===== Private Helper methods
    // Check Area Switch Point
    bool LocalPlanningManagerComponent::shouldSwitchArea(double x, double y)
    {
        /* should switch area A_i to A_(i+1) if...
                norm between current x,y and switch point P is less than threshold
        */
        auto [switch_x, switch_y, threshold] = area_switch_conditions_[current_area_id_];
        return isLessThanThreshold(x, y, switch_x, switch_y, threshold);
    }

    // Set or Chenge Strategy by Area ID
    void LocalPlanningManagerComponent::setAreaStrategy(const std::string &area_id)
    {
        // mapから該当する戦略を取得（O(1)）
        auto iter = area_strategies_.find(area_id);

        if (iter != area_strategies_.end())
        {
            current_area_strategy_ = iter->second.get();
        }
        else
        {
            current_area_strategy_ = nullptr;
            std::cerr << "Warning: Strategy not found for area: " << area_id << std::endl;
        }
    }



    // ===== Static Methods =====
    bool LocalPlanningManagerComponent::isLessThanThreshold(double x, double y, double target_x, double target_y, double threshold)
    {
        // return true if norm between target and current point is less than threshold
        return calculateDistance(x, y, target_x, target_y) <= threshold;
    }

    double LocalPlanningManagerComponent::calculateDistance(double x, double y, double target_x, double target_y)
    {
        return std::sqrt((target_x - x) * (target_x - x) + (target_y - y) * (target_y - y));
    }

} // namespace local_planning_manager
