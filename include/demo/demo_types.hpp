#pragma once
#include <string>
#include <unordered_map>

namespace demo
{

/**
 * @brief デモ用の遷移コンテキスト
 * 
 * ロボットの現在状態と位置情報を保持
 */
struct DemoTransitionContext
{
    std::string current_state;  // 現在の状態 ("StraightDrive", "SlowDrive", "Stop")
    double current_x;           // 現在のX座標
    double current_y;           // 現在のY座標
    double elapsed_time;        // 現在の状態になってからの経過時間
};

/**
 * @brief デモ用のセマンティック状態
 * 
 * 各ライフサイクルノードの状態を表現
 */
struct DemoSemanticState
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

    bool operator==(const DemoSemanticState &other) const noexcept
    {
        return semantic_state == other.semantic_state;
    }
};

/**
 * @brief デモ用のアクションステップ
 * 
 * ライフサイクルノードの操作を表現
 */
struct DemoActionStep
{
    std::string target_node_name;  // "straight_drive_node" or "slow_drive_node" or "stop_node"
    std::string operation;         // "configure" / "activate" / "deactivate" / "cleanup"
    double timeout_s{5.0};
    int retry{0};
};

/**
 * @brief デモ用の遷移レシピ
 * 
 * ひとつのエッジに対応する操作手順
 */
struct DemoTransitionRecipe
{
    std::vector<DemoActionStep> steps;
    std::string description;
};

/**
 * @brief エリアID定義
 */
namespace AreaID
{
    constexpr const char* AREA_A = "AreaA";
    constexpr const char* AREA_B = "AreaB";
    constexpr const char* AREA_C = "AreaC";
}

/**
 * @brief 状態ID定義
 */
namespace StateID
{
    constexpr const char* STRAIGHT_DRIVE = "StraightDrive";
    constexpr const char* SLOW_DRIVE = "SlowDrive";
    constexpr const char* STOP = "Stop";
}

} // namespace demo
