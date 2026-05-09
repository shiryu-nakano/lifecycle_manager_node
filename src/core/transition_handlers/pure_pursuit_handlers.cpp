#pragma once
#include "../../../include/local_planning_manager/core/transition_handler.hpp"
#include "../../../include/local_planning_manager/common_types.hpp"

namespace local_planning_manager
{

class PurePursuitToStoppingHandler : public TransitionHandler {
private:
    double threshold_;

public:
    explicit PurePursuitToStoppingHandler(double threshold = 0.5)
        : threshold_(threshold) {}

    std::optional<TransitionRecipe> handle(const TransitionContext& ctx) override {
        if (ctx.current_state == "PurePursuit" &&
            ctx.obstacle_detected &&
            ctx.obstacle_distance <= threshold_) {
            
            TransitionRecipe recipe;
            recipe.description = "PurePursuit to Stopping transition";
            
            ActionStep deactivate_pure_pursuit;
            deactivate_pure_pursuit.target_node_name = "pure_pursuit_node";
            deactivate_pure_pursuit.operation = "deactivate";
            deactivate_pure_pursuit.timeout_s = 5.0;
            deactivate_pure_pursuit.retry = 0;
            recipe.steps.push_back(deactivate_pure_pursuit);
            
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

class PurePursuitToDwaHandler : public TransitionHandler {
private:
    double threshold_;

public:
    explicit PurePursuitToDwaHandler(double threshold = 1.0)
        : threshold_(threshold) {}

    std::optional<TransitionRecipe> handle(const TransitionContext& ctx) override {
        if (ctx.current_state == "PurePursuit" &&
            ctx.obstacle_detected &&
            ctx.obstacle_distance <= threshold_) {
            
            TransitionRecipe recipe;
            recipe.description = "PurePursuit to DWA transition";
            
            ActionStep deactivate_pure_pursuit;
            deactivate_pure_pursuit.target_node_name = "pure_pursuit_node";
            deactivate_pure_pursuit.operation = "deactivate";
            deactivate_pure_pursuit.timeout_s = 5.0;
            deactivate_pure_pursuit.retry = 0;
            recipe.steps.push_back(deactivate_pure_pursuit);
            
            ActionStep activate_dwa;
            activate_dwa.target_node_name = "dwa_node";
            activate_dwa.operation = "activate";
            activate_dwa.timeout_s = 5.0;
            activate_dwa.retry = 0;
            recipe.steps.push_back(activate_dwa);
            
            return recipe;
        }
        return TransitionHandler::handle(ctx);
    }
};

} // namespace local_planning_manager
