#pragma once
#include "../transition_handler.hpp"

namespace local_planning_manager
{

class DwaToStoppingHandler : public TransitionHandler {
private:
    double threshold_;

public:
    explicit DwaToStoppingHandler(double threshold = 0.5)
        : threshold_(threshold) {}

    std::optional<TransitionRecipe> handle(const TransitionContext& ctx) override {
        if (ctx.current_state == "DWA" &&
            ctx.obstacle_detected &&
            ctx.obstacle_distance <= threshold_) {
            
            TransitionRecipe recipe;
            recipe.description = "DWA to Stopping transition";
            
            ActionStep deactivate_dwa;
            deactivate_dwa.target_node_name = "dwa_node";
            deactivate_dwa.operation = "deactivate";
            deactivate_dwa.timeout_s = 5.0;
            deactivate_dwa.retry = 0;
            recipe.steps.push_back(deactivate_dwa);
            
            ActionStep activate_stopping;
            activate_stopping.target_node_name = "stop_motion_node";
            activate_stopping.operation = "activate";
            activate_stopping.timeout_s = 5.0;
            activate_stopping.retry = 0;
            recipe.steps.push_back(activate_stopping);
            
            return recipe;
        }
        return TransitionHandler::handle(ctx);
    }
};

class DwaToPurePursuitHandler : public TransitionHandler {
public:
    std::optional<TransitionRecipe> handle(const TransitionContext& ctx) override {
        if (ctx.current_state == "DWA" && !ctx.obstacle_detected) {
            
            TransitionRecipe recipe;
            recipe.description = "DWA to PurePursuit transition";
            
            ActionStep deactivate_dwa;
            deactivate_dwa.target_node_name = "dwa_node";
            deactivate_dwa.operation = "deactivate";
            deactivate_dwa.timeout_s = 5.0;
            deactivate_dwa.retry = 0;
            recipe.steps.push_back(deactivate_dwa);
            
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

} // namespace local_planning_manager
