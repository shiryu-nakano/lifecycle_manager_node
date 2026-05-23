#pragma once
#include <memory>
#include <string>
#include <optional>
#include <vector>
#include <map>
#include <chrono>


#include "transition_recipe_test/common_types.hpp"
#include "transition_recipe_test/graph.hpp"
#include "transition_recipe_test/core/strategy.hpp"


namespace transition_recipe_test
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

    // ===== 設定 API（initialize の前に呼ぶ） =====

    /// エリアの並び順を指定する。先頭が初期エリアとして使われる。
    void setAreaList(const std::vector<std::string>& area_list);

    /// area_id から次エリアへ切り替える条件 (target_x, target_y, threshold) を登録する。
    void registerAreaSwitchCondition(
        const std::string& area_id, double target_x, double target_y, double threshold);

    /// area_id に対応する戦略を登録する。Componentが所有権を引き取る。
    void registerAreaStrategy(const std::string& area_id, std::unique_ptr<AreaStrategy> strategy);

    /// セマンティック状態辞書を設定する（getCurrentState の照合に使用）
    void setStateDictionary(std::unordered_map<std::string, SemanticState> state_dictionary);

    /// 設定完了後に呼ぶ。初期エリアの戦略を選択する。
    void initialize();

    // ===== 動作 API =====
    std::optional<TransitionRecipe> decideTransition(const TransitionContext& context);
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

    // 内部ヘルパー
    void setAreaStrategy(const std::string& area_id);
    bool shouldSwitchArea(double x, double y);

    static bool isLessThanThreshold(double x, double y, double target_x, double target_y, double threshold );
    static double calculateDistance(double x, double y, double target_x, double target_y);

    //さらにグラフを持つ
    Graph state_graph_;
};

} // namespace transition_recipe_test