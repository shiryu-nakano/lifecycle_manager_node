#pragma once
#include "demo_types.hpp"
#include "demo_transition_handler.hpp"
#include <memory>
#include <string>
#include <optional>

namespace demo
{

/**
 * @brief Strategy パターンの基底クラス
 * 
 * 各エリアの遷移戦略を定義
 */
class DemoAreaStrategy
{
protected:
    std::shared_ptr<DemoTransitionHandler> handler_chain_;

public:
    virtual ~DemoAreaStrategy() = default;

    virtual std::string get_area_name() const = 0;
    std::optional<DemoTransitionRecipe> processTransitions(const DemoTransitionContext& ctx);
};

/**
 * @brief エリアA戦略
 * 
 * X < 5.0: 直進エリア
 * 遷移: StraightDrive -> SlowDrive
 */
class AreaAStrategy : public DemoAreaStrategy
{
public:
    AreaAStrategy();
    std::string get_area_name() const override;
};

/**
 * @brief エリアB戦略
 * 
 * 5.0 <= X < 10.0: ゆっくり移動エリア
 * 遷移: SlowDrive -> Stop
 */
class AreaBStrategy : public DemoAreaStrategy
{
public:
    AreaBStrategy();
    std::string get_area_name() const override;
};

/**
 * @brief エリアC戦略
 * 
 * X >= 10.0: 停止エリア
 * 遷移なし
 */
class AreaCStrategy : public DemoAreaStrategy
{
public:
    AreaCStrategy();
    std::string get_area_name() const override;
};

} // namespace demo
