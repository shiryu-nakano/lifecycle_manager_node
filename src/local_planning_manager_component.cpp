#include "transition_recipe_test/local_planning_manager_component.hpp"
#include "transition_recipe_test/core/strategy.hpp"
#include <cmath>
#include <iostream>

namespace transition_recipe_test
{

LocalPlanningManagerComponent::LocalPlanningManagerComponent()
    : current_area_index_(0), current_area_strategy_(nullptr)
{
}

// ===== 設定 API =====
void LocalPlanningManagerComponent::setAreaList(const std::vector<std::string> &area_list)
{
    area_list_ = area_list;
    current_area_index_ = 0;
    if (!area_list_.empty())
    {
        current_area_id_ = area_list_[0];
    }
}

void LocalPlanningManagerComponent::registerAreaSwitchCondition(
    const std::string &area_id, double target_x, double target_y, double threshold)
{
    area_switch_conditions_[area_id] = std::make_tuple(target_x, target_y, threshold);
}

void LocalPlanningManagerComponent::registerAreaStrategy(
    const std::string &area_id, std::unique_ptr<AreaStrategy> strategy)
{
    area_strategies_[area_id] = std::move(strategy);
}

void LocalPlanningManagerComponent::setStateDictionary(
    std::unordered_map<std::string, SemanticState> state_dictionary)
{
    state_graph_ = Graph(std::move(state_dictionary));
}

void LocalPlanningManagerComponent::initialize()
{
    if (area_list_.empty())
    {
        std::cerr << "Warning: area_list is empty; LocalPlanningManagerComponent has no current strategy."
                  << std::endl;
        current_area_strategy_ = nullptr;
        return;
    }
    current_area_index_ = 0;
    current_area_id_ = area_list_[0];
    setAreaStrategy(current_area_id_);
}

// ===== 動作 API =====
std::optional<std::string> LocalPlanningManagerComponent::getCurrentState(SemanticState semantic) const
{
    return state_graph_.getCurrentSemanticState(semantic);
}

std::optional<TransitionRecipe> LocalPlanningManagerComponent::decideTransition(const TransitionContext &context)
{
    // 最終エリアに到達していない場合のみエリア切替を検討
    if (current_area_index_ < static_cast<int>(area_list_.size()) - 1)
    {
        if (shouldSwitchArea(context.current_x, context.current_y))
        {
            const std::string pre_area_id = area_list_[current_area_index_];
            current_area_index_++;
            current_area_id_ = area_list_[current_area_index_];
            setAreaStrategy(current_area_id_);
            std::cout << "Area switched from " << pre_area_id << " to: " << current_area_id_ << std::endl;
        }
    }

    if (current_area_strategy_ != nullptr)
    {
        return current_area_strategy_->processTransitions(context);
    }
    return std::nullopt;
}

// ===== 内部ヘルパー =====
bool LocalPlanningManagerComponent::shouldSwitchArea(double x, double y)
{
    auto it = area_switch_conditions_.find(current_area_id_);
    if (it == area_switch_conditions_.end())
    {
        return false;
    }
    auto [switch_x, switch_y, threshold] = it->second;
    return isLessThanThreshold(x, y, switch_x, switch_y, threshold);
}

void LocalPlanningManagerComponent::setAreaStrategy(const std::string &area_id)
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

// ===== Static Methods =====
bool LocalPlanningManagerComponent::isLessThanThreshold(
    double x, double y, double target_x, double target_y, double threshold)
{
    return calculateDistance(x, y, target_x, target_y) <= threshold;
}

double LocalPlanningManagerComponent::calculateDistance(
    double x, double y, double target_x, double target_y)
{
    return std::sqrt((target_x - x) * (target_x - x) + (target_y - y) * (target_y - y));
}

} // namespace transition_recipe_test
