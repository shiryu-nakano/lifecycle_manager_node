#pragma once
#include "../common_types.hpp"
#include <memory>
#include <optional>

namespace local_planning_manager
{

// * @brief Chain of Responsibility パターンの基底クラス
class TransitionHandler {
protected:
    std::shared_ptr<TransitionHandler> next_;

public:
    virtual ~TransitionHandler() = default;

    void set_next(std::shared_ptr<TransitionHandler> handler) {
        next_ = handler;
    }

    virtual std::optional<TransitionRecipe> handle(const TransitionContext& ctx) {
        if (next_) {
            return next_->handle(ctx);
        }
        return std::nullopt;
    }
};

} // namespace local_planning_manager
