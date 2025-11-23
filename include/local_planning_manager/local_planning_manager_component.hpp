#pragma once
#include <memory>
#include <string>
#include <optional>
#include <vector>
#include <map>
#include <chrono>


#include "local_planning_manager/common_types.hpp" 
#include "local_planning_manager/core/graph.hpp"
#include "local_planning_manager/core/strategy.hpp"


namespace local_planning_manager
{

// 前方宣言
class AreaStrategy;

/**
 * @brief 状態遷移管理のロジッククラス（ROS非依存）
 * 
 * NodeはこのComponentを通じてのみ状態遷移ロジックにアクセスする。
 * Strategy/Handler/CoRパターンの実装詳細はprivateメンバとして隠蔽される。
 */
class LocalPlanningManagerComponent {
public:
    LocalPlanningManagerComponent();
    ~LocalPlanningManagerComponent() = default;

    std::optional<TransitionRecipe> decideTransition(const TransitionContext& context);

    //std::string getCurrentArea(double x,double y);

    std::optional<std::string> getCurrentState(SemanticState semantic) const;

private:
    // エリア管理
    std::vector<std::string> area_list_;
    std::unordered_map<std::string, std::tuple<double, double, double>> area_switch_conditions_;
    
    int         current_area_index_;
    std::string current_area_id_;

    // Strategy of Chain of Responsibility
    std::unordered_map<std::string, std::unique_ptr<AreaStrategy> > area_strategies_;
    AreaStrategy* current_area_strategy_;  // Non-owning pointer
    
    // 初期化メソッド
    void initializeStrategies();
    void setAreaList(const std::vector<std::string>& area_list);
    void registerAreaSwitchCondition(const std::string& area_id, double target_x, double target_y, double threshold);
    
    // 内部ヘルパー
    void setAreaStrategy(const std::string& area_id);
    bool shouldSwitchArea(double x, double y);

    

    static bool isLessThanThreshold(double x, double y, double target_x, double target_y, double threshold );
    static double calculateDistance(double x, double y, double target_x, double target_y);

    //さらにグラフを持つ
    Graph state_graph_;

};

} // namespace local_planning_manager