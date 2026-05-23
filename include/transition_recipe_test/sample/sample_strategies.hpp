#pragma once
#include "transition_recipe_test/core/strategy.hpp"
#include "transition_recipe_test/sample/sample_handlers.hpp"

namespace transition_recipe_test
{
namespace sample
{

/// サンプルの状態名（StateID）
namespace SampleStateID
{
inline constexpr const char *STRAIGHT_DRIVE = "StraightDrive";
inline constexpr const char *SLOW_DRIVE = "SlowDrive";
inline constexpr const char *STOP = "Stop";
} // namespace SampleStateID

/// 状態名 → ライフサイクルノード名のサンプル対応表を返すヘルパ
inline std::unordered_map<std::string, std::string> sampleStateToNodeMap()
{
    return {
        {SampleStateID::STRAIGHT_DRIVE, "straight_drive_node"},
        {SampleStateID::SLOW_DRIVE,     "slow_drive_node"},
        {SampleStateID::STOP,           "stop_node"},
    };
}

/**
 * @brief サンプル用 Area A 戦略：StraightDrive 状態を維持する。
 */
class SampleAreaAStrategy : public AreaStrategy
{
public:
    SampleAreaAStrategy()
    {
        handler_chain_ = std::make_shared<EnsureStateHandler>(
            SampleStateID::STRAIGHT_DRIVE, sampleStateToNodeMap());
    }
    std::string get_area_name() const override { return "A"; }
};

/**
 * @brief サンプル用 Area B 戦略：SlowDrive 状態を維持する。
 */
class SampleAreaBStrategy : public AreaStrategy
{
public:
    SampleAreaBStrategy()
    {
        handler_chain_ = std::make_shared<EnsureStateHandler>(
            SampleStateID::SLOW_DRIVE, sampleStateToNodeMap());
    }
    std::string get_area_name() const override { return "B"; }
};

/**
 * @brief サンプル用 Area C 戦略：Stop 状態を維持する。
 */
class SampleAreaCStrategy : public AreaStrategy
{
public:
    SampleAreaCStrategy()
    {
        handler_chain_ = std::make_shared<EnsureStateHandler>(
            SampleStateID::STOP, sampleStateToNodeMap());
    }
    std::string get_area_name() const override { return "C"; }
};

} // namespace sample
} // namespace transition_recipe_test
