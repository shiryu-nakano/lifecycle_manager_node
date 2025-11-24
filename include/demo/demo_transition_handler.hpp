#pragma once
#include "demo_types.hpp"
#include <memory>
#include <optional>

namespace demo
{

/**
 * @brief Chain of Responsibility パターンの基底クラス
 * 
 * 遷移条件をチェックし、条件に合致すればTransitionRecipeを返す
 */
class DemoTransitionHandler
{
protected:
    std::shared_ptr<DemoTransitionHandler> next_;

public:
    virtual ~DemoTransitionHandler() = default;

    void set_next(std::shared_ptr<DemoTransitionHandler> handler);
    virtual std::optional<DemoTransitionRecipe> handle(const DemoTransitionContext& ctx);
};

/**
 * @brief StraightDrive -> SlowDrive 遷移ハンドラー
 * 
 * 条件: X座標が閾値を超えた時
 */
class StraightToSlowHandler : public DemoTransitionHandler
{
private:
    double threshold_x_;

public:
    explicit StraightToSlowHandler(double threshold_x = 5.0);
    std::optional<DemoTransitionRecipe> handle(const DemoTransitionContext& ctx) override;
};

/**
 * @brief SlowDrive -> Stop 遷移ハンドラー
 * 
 * 条件: X座標が閾値を超えた時
 */
class SlowToStopHandler : public DemoTransitionHandler
{
private:
    double threshold_x_;

public:
    explicit SlowToStopHandler(double threshold_x = 10.0);
    std::optional<DemoTransitionRecipe> handle(const DemoTransitionContext& ctx) override;
};

} // namespace demo
