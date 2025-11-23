#pragma once
#include "local_planning_manager/core/strategy.hpp"
#include "local_planning_manager/core/transition_handlers/pure_pursuit_handlers.hpp"
#include "local_planning_manager/core/transition_handlers/dwa_handlers.hpp"
#include "local_planning_manager/core/transition_handlers/stopping_handlers.hpp"
#include "local_planning_manager/core/transition_handlers/inplace_turn_handlers.hpp"

/*
Area A Strategy:
→参照




*/
namespace local_planning_manager
{

class AreaAStrategy : public AreaStrategy {
public:
    AreaAStrategy() {
        auto h1 = std::make_shared<PurePursuitToStoppingHandler>(0.5);
        auto h2 = std::make_shared<PurePursuitToDwaHandler>(1.0);
        auto h3 = std::make_shared<DwaToStoppingHandler>(0.5);
        auto h4 = std::make_shared<DwaToPurePursuitHandler>();
        auto h5 = std::make_shared<StoppingToPurePursuitHandler>();
        auto h6 = std::make_shared<StoppingToInplaceTurnHandler>(5.0);
        auto h7 = std::make_shared<InplaceTurnToDwaHandler>(1.0);

        h1->set_next(h2);
        h2->set_next(h3);
        h3->set_next(h4);
        h4->set_next(h5);
        h5->set_next(h6);
        h6->set_next(h7);

        handler_chain_ = h1;
    }

    std::string get_area_name() const override { return "A"; }
};

} // namespace local_planning_manager
