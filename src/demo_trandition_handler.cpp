#include "demo/demo_transition_handler.hpp"
#include <iostream>
#include <cstring>

namespace demo
{

void DemoTransitionHandler::set_next(std::shared_ptr<DemoTransitionHandler> handler)
{
    next_ = handler;
}

std::optional<DemoTransitionRecipe> DemoTransitionHandler::handle(const DemoTransitionContext& ctx)
{
    if (next_)
    {
        return next_->handle(ctx);
    }
    return std::nullopt;
}

// StraightToSlowHandler Implementation
StraightToSlowHandler::StraightToSlowHandler(double threshold_x)
    : threshold_x_(threshold_x) {}

std::optional<DemoTransitionRecipe> StraightToSlowHandler::handle(const DemoTransitionContext& ctx)
{
    std::cout << "[StraightToSlowHandler] Checking: current_state='" << ctx.current_state 
              << "' (len=" << ctx.current_state.length() << "), x=" << ctx.current_x 
              << ", threshold=" << threshold_x_ << std::endl;
    std::cout << "[StraightToSlowHandler] StateID::STRAIGHT_DRIVE='" << StateID::STRAIGHT_DRIVE 
              << "' (len=" << std::strlen(StateID::STRAIGHT_DRIVE) << ")" << std::endl;
    std::cout << "[StraightToSlowHandler] String match: " << (ctx.current_state == StateID::STRAIGHT_DRIVE ? "YES" : "NO")
              << ", x >= threshold: " << (ctx.current_x >= threshold_x_ ? "YES" : "NO") << std::endl;
    
    if (ctx.current_state == StateID::STRAIGHT_DRIVE && ctx.current_x >= threshold_x_)
    {
        std::cout << "[StraightToSlowHandler] Transition condition MET!" << std::endl;
        
        DemoTransitionRecipe recipe;
        recipe.description = "StraightDrive to SlowDrive transition";

        // straight_drive_node を deactivate
        DemoActionStep deactivate_straight;
        deactivate_straight.target_node_name = "straight_drive_node";
        deactivate_straight.operation = "deactivate";
        deactivate_straight.timeout_s = 5.0;
        recipe.steps.push_back(deactivate_straight);

        // slow_drive_node を activate
        DemoActionStep activate_slow;
        activate_slow.target_node_name = "slow_drive_node";
        activate_slow.operation = "activate";
        activate_slow.timeout_s = 5.0;
        recipe.steps.push_back(activate_slow);

        return recipe;
    }
    return DemoTransitionHandler::handle(ctx);
}

// SlowToStopHandler Implementation
SlowToStopHandler::SlowToStopHandler(double threshold_x)
    : threshold_x_(threshold_x) {}

std::optional<DemoTransitionRecipe> SlowToStopHandler::handle(const DemoTransitionContext& ctx)
{
    std::cout << "[SlowToStopHandler] Checking: current_state='" << ctx.current_state 
              << "' (len=" << ctx.current_state.length() << "), x=" << ctx.current_x 
              << ", threshold=" << threshold_x_ << std::endl;
    std::cout << "[SlowToStopHandler] StateID::SLOW_DRIVE='" << StateID::SLOW_DRIVE 
              << "' (len=" << std::strlen(StateID::SLOW_DRIVE) << ")" << std::endl;
    std::cout << "[SlowToStopHandler] String match: " << (ctx.current_state == StateID::SLOW_DRIVE ? "YES" : "NO")
              << ", x >= threshold: " << (ctx.current_x >= threshold_x_ ? "YES" : "NO") << std::endl;
    
    if (ctx.current_state == StateID::SLOW_DRIVE && ctx.current_x >= threshold_x_)
    {
        std::cout << "[SlowToStopHandler] Transition condition MET!" << std::endl;
        
        DemoTransitionRecipe recipe;
        recipe.description = "SlowDrive to Stop transition";

        // slow_drive_node を deactivate
        DemoActionStep deactivate_slow;
        deactivate_slow.target_node_name = "slow_drive_node";
        deactivate_slow.operation = "deactivate";
        deactivate_slow.timeout_s = 5.0;
        recipe.steps.push_back(deactivate_slow);

        // stop_node を activate
        DemoActionStep activate_stop;
        activate_stop.target_node_name = "stop_node";
        activate_stop.operation = "activate";
        activate_stop.timeout_s = 5.0;
        recipe.steps.push_back(activate_stop);

        return recipe;
    }
    std::cout << "[SlowToStopHandler] Transition condition NOT met, calling next handler" << std::endl;
    return DemoTransitionHandler::handle(ctx);
}

} // namespace demo
