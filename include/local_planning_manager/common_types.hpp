#pragma once
#include <string>
#include <functional>
#include <unordered_map>
#include <vector>
#include <optional>

namespace local_planning_manager
{

    struct TransitionContext
    {
        std::string current_state;
        bool obstacle_detected;
        double obstacle_distance;
        double elapsed_time;
        double current_x; // 追加
        double current_y; // 追加
    };

    // ライフサイクルノードの状態の直積でモードを表現する
    // 値一致とする
    struct SemanticState
    {
        enum class State : std::uint8_t
        {
            UNKNOWN = 0,
            UNCONFIGURED = 1,
            INACTIVE = 2,
            ACTIVE = 3,
            FINALIZED = 4
        };

        std::unordered_map<std::string, State> semantic_state;

        // 値一致
        bool operator==(const SemanticState &other) const noexcept
        {
            return semantic_state == other.semantic_state;
        }
    };

    // アクション手順（Lifecycleノード操作）
    struct ActionStep
    {
        std::string target_node_name; // "gnss" or "emcl" or somethig
        std::string operation;        // "configure" / "activate" / "deactivate"
        double timeout_s{5.0};
        int retry{0};
    };

    // 遷移レシピ（ひとつのエッジに対応）
    struct TransitionRecipe
    {
        std::vector<ActionStep> steps;
        std::string description;
    };

} // namespace local_planning_manager
