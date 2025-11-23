#pragma once
#include "../transition_handler.hpp"

namespace local_planning_manager
{

class StoppingToPurePursuitHandler : public TransitionHandler {
public:
    std::optional<TransitionRecipe> handle(const TransitionContext& ctx) override {
        if (ctx.current_state == "Stopping" && !ctx.obstacle_detected) {
            
            TransitionRecipe recipe;
            recipe.description = "Stopping to PurePursuit transition";
            
            ActionStep deactivate_stopping;
            deactivate_stopping.target_node_name = "stop_motion_node";
            deactivate_stopping.operation = "deactivate";
            deactivate_stopping.timeout_s = 5.0;
            deactivate_stopping.retry = 0;
            recipe.steps.push_back(deactivate_stopping);
            
            ActionStep activate_pure_pursuit;
            activate_pure_pursuit.target_node_name = "pure_pursuit_node";
            activate_pure_pursuit.operation = "activate";
            activate_pure_pursuit.timeout_s = 5.0;
            activate_pure_pursuit.retry = 0;
            recipe.steps.push_back(activate_pure_pursuit);
            
            return recipe;
        }
        return TransitionHandler::handle(ctx);
    }
};

class StoppingToInplaceTurnHandler : public TransitionHandler {
private:
    double timeout_;

public:
    explicit StoppingToInplaceTurnHandler(double timeout = 5.0)
        : timeout_(timeout) {}

    std::optional<TransitionRecipe> handle(const TransitionContext& ctx) override {
        if (ctx.current_state == "Stopping" && 
            ctx.obstacle_detected &&
            ctx.elapsed_time >= timeout_) {
            
            TransitionRecipe recipe;
            recipe.description = "Stopping to InplaceTurn transition";
            
            ActionStep deactivate_stopping;
            deactivate_stopping.target_node_name = "stop_motion_node";
            deactivate_stopping.operation = "deactivate";
            deactivate_stopping.timeout_s = 5.0;
            deactivate_stopping.retry = 0;
            recipe.steps.push_back(deactivate_stopping);
            
            ActionStep activate_inplace_turn;
            activate_inplace_turn.target_node_name = "in_place_turn_node";
            activate_inplace_turn.operation = "activate";
            activate_inplace_turn.timeout_s = 5.0;
            activate_inplace_turn.retry = 0;
            recipe.steps.push_back(activate_inplace_turn);
            
            return recipe;
        }
        return TransitionHandler::handle(ctx);
    }
};

} // namespace local_planning_manager
