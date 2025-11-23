#pragma once
#include "transition_handler.hpp"
#include <memory>
#include <string>
#include <optional>

namespace local_planning_manager
{

/**
 * @brief Strategy パターンの基底クラス（ROS非依存）
 */
class AreaStrategy {
protected:
    std::shared_ptr<TransitionHandler> handler_chain_;

public:
    virtual ~AreaStrategy() = default;

    virtual std::string get_area_name() const = 0;

    std::optional<TransitionRecipe> processTransitions(const TransitionContext& ctx) {
        if (handler_chain_) {
            return handler_chain_->handle(ctx);
        }
        return std::nullopt;
    }
};

} // namespace local_planning_manager
