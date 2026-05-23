#pragma once
#include "transition_recipe_test/common_types.hpp"
#include "transition_recipe_test/core/transition_handler.hpp"
#include <unordered_map>

namespace transition_recipe_test
{
namespace sample
{

/**
 * @brief サンプル用のシンプルな TransitionHandler。
 *
 * 「望ましい状態 (target_state)」と「状態名 → ライフサイクルノード名」の
 * 対応表を持ち、現在状態が望ましい状態と異なるなら現状態のノードを
 * deactivate し、目標状態のノードを activate するレシピを返す。
 *
 * パッケージを使う側が独自のハンドラを書くときの最小例として参考にできる。
 */
class EnsureStateHandler : public TransitionHandler
{
public:
    EnsureStateHandler(std::string target_state,
                       std::unordered_map<std::string, std::string> state_to_node)
        : target_state_(std::move(target_state)),
          state_to_node_(std::move(state_to_node))
    {
    }

    std::optional<TransitionRecipe> handle(const TransitionContext &ctx) override
    {
        if (ctx.current_state == target_state_)
        {
            // 既に望ましい状態。次のハンドラへ。
            return TransitionHandler::handle(ctx);
        }

        TransitionRecipe recipe;
        recipe.description = "Ensure state: " + ctx.current_state + " -> " + target_state_;

        // 現在状態のノードを deactivate（マッピングがあれば）
        auto cur_it = state_to_node_.find(ctx.current_state);
        if (cur_it != state_to_node_.end())
        {
            ActionStep step;
            step.target_node_name = cur_it->second;
            step.operation = "deactivate";
            step.timeout_s = 2.0;
            recipe.steps.push_back(step);
        }

        // 目標状態のノードを activate
        auto tgt_it = state_to_node_.find(target_state_);
        if (tgt_it != state_to_node_.end())
        {
            ActionStep step;
            step.target_node_name = tgt_it->second;
            step.operation = "activate";
            step.timeout_s = 2.0;
            recipe.steps.push_back(step);
        }

        return recipe;
    }

private:
    std::string target_state_;
    std::unordered_map<std::string, std::string> state_to_node_;
};

} // namespace sample
} // namespace transition_recipe_test
