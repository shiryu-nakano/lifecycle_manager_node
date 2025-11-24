#pragma once
#include "local_planning_manager/core/transition_handler.hpp"

namespace local_planning_manager
{

/**
 * @brief Stopping → PurePursuit 遷移ハンドラー
 */
class StoppingToPurePursuitHandler : public TransitionHandler {
public:
    std::optional<TransitionRecipe> handle(const TransitionContext& ctx) override;
};

/**
 * @brief Stopping → InplaceTurn 遷移ハンドラー
 */
class StoppingToInplaceTurnHandler : public TransitionHandler {
private:
    double timeout_;

public:
    explicit StoppingToInplaceTurnHandler(double timeout = 5.0);
    std::optional<TransitionRecipe> handle(const TransitionContext& ctx) override;
};

} // namespace local_planning_manager
