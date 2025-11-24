#pragma once
#include "local_planning_manager/core/transition_handler.hpp"

namespace local_planning_manager
{

/**
 * @brief InplaceTurn → DWA 遷移ハンドラー
 */
class InplaceTurnToDwaHandler : public TransitionHandler {
private:
    double timeout_;

public:
    explicit InplaceTurnToDwaHandler(double timeout = 1.0);
    std::optional<TransitionRecipe> handle(const TransitionContext& ctx) override;
};

} // namespace local_planning_manager
