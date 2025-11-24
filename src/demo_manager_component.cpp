#include "demo/demo_manager_component.hpp"
#include <iostream>
#include <cmath>

namespace demo
{

DemoManagerComponent::DemoManagerComponent()
    : current_area_index_(0), current_area_strategy_(nullptr)
{
    // エリアリストの設定
    setAreaList({AreaID::AREA_A, AreaID::AREA_B, AreaID::AREA_C});

    // エリアの切り替え条件（本番と同じ: target_x, target_y, threshold）
    registerAreaSwitchCondition(AreaID::AREA_A, 5.0, 0.0, 0.5);   // A to B: point(5.0, 0.0), threshold=0.5
    registerAreaSwitchCondition(AreaID::AREA_B, 10.0, 0.0, 0.5);  // B to C: point(10.0, 0.0), threshold=0.5

    // 戦略の初期化
    area_strategies_[AreaID::AREA_A] = std::make_unique<AreaAStrategy>();
    area_strategies_[AreaID::AREA_B] = std::make_unique<AreaBStrategy>();
    area_strategies_[AreaID::AREA_C] = std::make_unique<AreaCStrategy>();

    // 初期エリアの戦略を設定
    if (!area_list_.empty())
    {
        current_area_id_ = area_list_[current_area_index_];
        setAreaStrategy(current_area_id_);
    }
}

void DemoManagerComponent::setAreaList(const std::vector<std::string>& area_list)
{
    area_list_ = area_list;
    current_area_index_ = 0;
    if (!area_list_.empty())
    {
        current_area_id_ = area_list_[0];
    }
}

void DemoManagerComponent::registerAreaSwitchCondition(const std::string& area_id, double target_x, double target_y, double threshold)
{
    area_switch_conditions_[area_id] = std::make_tuple(target_x, target_y, threshold);
}

std::optional<DemoTransitionRecipe> DemoManagerComponent::decideTransition(const DemoTransitionContext& context)
{
    std::cout << "[DemoComponent] decideTransition called: state=" << context.current_state 
              << ", x=" << context.current_x << ", y=" << context.current_y 
              << ", current_area=" << current_area_id_ << std::endl;

    // 最終エリアに到達していない時
    if (current_area_index_ < static_cast<int>(area_list_.size()) - 1)
    {
        std::cout << "[DemoComponent] Checking area switch: current_index=" << current_area_index_ 
                  << ", area_list_.size()=" << area_list_.size() << std::endl;
        
        if (shouldSwitchArea(context.current_x, context.current_y))
        {
            // エリアを更新
            std::string pre_area_id = area_list_[current_area_index_];
            current_area_index_++;
            current_area_id_ = area_list_[current_area_index_];
            
            // エリア戦略の切り替え
            setAreaStrategy(current_area_id_);

            std::cout << "[DemoComponent] Area switched from " << pre_area_id << " to: " << current_area_id_ << std::endl;
        }
    }

    // 現在のエリア戦略で遷移判定
    if (current_area_strategy_ != nullptr)
    {
        std::cout << "[DemoComponent] Calling strategy->processTransitions()" << std::endl;
        auto result = current_area_strategy_->processTransitions(context);
        if (result.has_value())
        {
            std::cout << "[DemoComponent] Transition recipe found: " << result.value().description << std::endl;
        }
        else
        {
            std::cout << "[DemoComponent] No transition recipe found" << std::endl;
        }
        return result;
    }

    std::cout << "[DemoComponent] No strategy available" << std::endl;
    return std::nullopt;
}

bool DemoManagerComponent::shouldSwitchArea(double x, double y)
{
    // 現在のエリアの切り替え条件を取得
    auto it = area_switch_conditions_.find(current_area_id_);
    if (it != area_switch_conditions_.end())
    {
        auto [switch_x, switch_y, threshold] = it->second;
        bool should_switch = isLessThanThreshold(x, y, switch_x, switch_y, threshold);
        std::cout << "[DemoComponent] shouldSwitchArea: current=(" << x << "," << y << "), "
                  << "target=(" << switch_x << "," << switch_y << "), threshold=" << threshold
                  << ", should_switch=" << should_switch << std::endl;
        return should_switch;
    }
    std::cout << "[DemoComponent] shouldSwitchArea: No condition found for " << current_area_id_ << std::endl;
    return false;
}

void DemoManagerComponent::setAreaStrategy(const std::string& area_id)
{
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

bool DemoManagerComponent::isLessThanThreshold(double x, double y, double target_x, double target_y, double threshold)
{
    return calculateDistance(x, y, target_x, target_y) <= threshold;
}

double DemoManagerComponent::calculateDistance(double x, double y, double target_x, double target_y)
{
    return std::sqrt((target_x - x) * (target_x - x) + (target_y - y) * (target_y - y));
}

} // namespace demo
