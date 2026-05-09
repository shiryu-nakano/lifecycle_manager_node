#pragma once
#include "../../../include/local_planning_manager/core/transition_handler.hpp"
#include "../../../include/local_planning_manager/common_types.hpp"

namespace local_planning_manager
{

class InplaceTurnToDwaHandler : public TransitionHandler {
private:
    double timeout_;

public:
    explicit InplaceTurnToDwaHandler(double timeout = 1.0)
        : timeout_(timeout) {}

    std::optional<TransitionRecipe> handle(const TransitionContext& ctx) override {
        if (ctx.current_state == "InplaceTurn" && 
            ctx.elapsed_time >= timeout_) {
            
            TransitionRecipe recipe;
            recipe.description = "InplaceTurn to DWA transition";
            
            ActionStep deactivate_inplace_turn;
            deactivate_inplace_turn.target_node_name = "in_place_turn_node";
            deactivate_inplace_turn.operation        = "deactivate";
            deactivate_inplace_turn.timeout_s        = 5.0;
            deactivate_inplace_turn.retry            = 0;
            recipe.steps.push_back(deactivate_inplace_turn);
            
            ActionStep activate_dwa;
            activate_dwa.target_node_name = "dwa_node";
            activate_dwa.operation        = "activate";
            activate_dwa.timeout_s        = 5.0;
            activate_dwa.retry            = 0;
            recipe.steps.push_back(activate_dwa);
            
            return recipe;
        }
        return TransitionHandler::handle(ctx);
    }
};

} // namespace local_planning_manager
