#pragma once
#include "local_planning_manager/core/strategy.hpp"
//#include "local_planning_manager/core/transition_handlers/pure_pursuit_handlers.hpp"
//#include "local_planning_manager/core/transition_handlers/stopping_handlers.hpp"
//#include "local_planning_manager/core/transition_handlers/inplace_turn_handlers.hpp"

namespace local_planning_manager
{

class DemoAreaBStrategy : public AreaStrategy {
public:
    AreaBStrategy() {
        /*
        auto h1 = std::make_shared<PurePursuitToStoppingHandler>(0.5);
        auto h2 = std::make_shared<StoppingToPurePursuitHandler>();
        auto h3 = std::make_shared<StoppingToInplaceTurnHandler>(10.0);

        h1->set_next(h2);
        h2->set_next(h3);

        handler_chain_ = h1;
        */
        }

    std::string get_area_name() const override { return "B"; }
};

} // namespace local_planning_manager
