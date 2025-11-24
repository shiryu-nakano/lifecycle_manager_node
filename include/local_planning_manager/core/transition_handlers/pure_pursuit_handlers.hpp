#pragma once
#include "local_planning_manager/core/transition_handler.hpp"

namespace local_planning_manager
{

/**
 * @brief PurePursuit → Stopping 遷移ハンドラー
 */
class PurePursuitToStoppingHandler : public TransitionHandler {
private:
    double threshold_;

public:
    explicit PurePursuitToStoppingHandler(double threshold = 0.5);
    std::optional<TransitionRecipe> handle(const TransitionContext& ctx) override;
};

/**
 * @brief PurePursuit → DWA 遷移ハンドラー
 */
class PurePursuitToDwaHandler : public TransitionHandler {
private:
    double threshold_;

public:
    explicit PurePursuitToDwaHandler(double threshold = 1.0);
    std::optional<TransitionRecipe> handle(const TransitionContext& ctx) override;
};

} // namespace local_planning_manager
