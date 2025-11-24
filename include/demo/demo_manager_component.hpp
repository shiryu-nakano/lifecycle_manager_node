#pragma once
#include <memory>
#include <string>
#include <optional>
#include <vector>
#include <map>
#include <chrono>
#include <unordered_map>

#include "demo/demo_types.hpp"
#include "demo/demo_strategy.hpp"

namespace demo
{

/**
 * @brief デモ用の状態遷移管理のロジッククラス（ROS非依存）
 * 
 * 本体のLocalPlanningManagerComponentと同じ構造
 */
class DemoManagerComponent
{
public:
    DemoManagerComponent();
    ~DemoManagerComponent() = default;

    std::optional<DemoTransitionRecipe> decideTransition(const DemoTransitionContext& context);
    std::optional<std::string> getCurrentArea() const { return current_area_id_; }
    std::optional<std::string> getCurrentArea() const { return current_area_id_; }

private:
    // エリア管理
    std::vector<std::string> area_list_;
    std::unordered_map<std::string, std::tuple<double, double, double>> area_switch_conditions_;
    
    int         current_area_index_;
    std::string current_area_id_;

    // Strategy of Chain of Responsibility
    std::unordered_map<std::string, std::unique_ptr<DemoAreaStrategy>> area_strategies_;
    DemoAreaStrategy* current_area_strategy_;  // Non-owning pointer
    
    // 初期化メソッド
    void setAreaList(const std::vector<std::string>& area_list);
    void registerAreaSwitchCondition(const std::string& area_id, double target_x, double target_y, double threshold);
    
    // 内部ヘルパー
    void setAreaStrategy(const std::string& area_id);
    bool shouldSwitchArea(double x, double y);
    
    // Static methods
    static bool isLessThanThreshold(double x, double y, double target_x, double target_y, double threshold);
    static double calculateDistance(double x, double y, double target_x, double target_y);
};

} // namespace demo
