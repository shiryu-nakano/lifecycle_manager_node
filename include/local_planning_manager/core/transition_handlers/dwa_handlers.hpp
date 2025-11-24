#pragma once
#include "local_planning_manager/core/transition_handler.hpp"

namespace local_planning_manager
{

/**
 * @brief DWA → Stopping 遷移ハンドラー
 * 障害物を検知し距離が閾値以下の場合に停止する
 */
class DwaToStoppingHandler : public TransitionHandler {
private:
    double threshold_;

public:
    explicit DwaToStoppingHandler(double threshold = 0.5);
    std::optional<TransitionRecipe> handle(const TransitionContext& ctx) override;
};

/**
 * @brief DWA → PurePursuit 遷移ハンドラー
 * 障害物が検知されなくなった場合にPurePursuitに戻る
 */
class DwaToPurePursuitHandler : public TransitionHandler {
public:
    std::optional<TransitionRecipe> handle(const TransitionContext& ctx) override;
};

} // namespace local_planning_manager
