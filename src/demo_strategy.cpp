#include "demo/demo_strategy.hpp"

namespace demo
{

// DemoAreaStrategy Implementation
std::optional<DemoTransitionRecipe> DemoAreaStrategy::processTransitions(const DemoTransitionContext& ctx)
{
    if (handler_chain_)
    {
        return handler_chain_->handle(ctx);
    }
    return std::nullopt;
}

// AreaAStrategy Implementation
AreaAStrategy::AreaAStrategy()
{
    // ハンドラーチェーンの構築
    // 切り替えポイント(5.0, 0.0)の手前で遷移（4.5付近）
    auto h1 = std::make_shared<StraightToSlowHandler>(4.5);
    handler_chain_ = h1;
}

std::string AreaAStrategy::get_area_name() const
{
    return AreaID::AREA_A;
}

// AreaBStrategy Implementation
AreaBStrategy::AreaBStrategy()
{
    // ハンドラーチェーンの構築
    // 切り替えポイント(10.0, 0.0)の手前で遷移（9.5付近）
    auto h1 = std::make_shared<SlowToStopHandler>(9.5);
    handler_chain_ = h1;
}

std::string AreaBStrategy::get_area_name() const
{
    return AreaID::AREA_B;
}

// AreaCStrategy Implementation
AreaCStrategy::AreaCStrategy()
{
    // 遷移なし（停止状態を維持）
    handler_chain_ = nullptr;
}

std::string AreaCStrategy::get_area_name() const
{
    return AreaID::AREA_C;
}

} // namespace demo
