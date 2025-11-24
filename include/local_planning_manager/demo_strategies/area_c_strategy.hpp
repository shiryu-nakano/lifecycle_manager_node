#pragma once
#include "local_planning_manager/core/strategy.hpp"

//#include "local_planning_manager/core/transition_handlers/pure_pursuit_handlers.hpp"
//#include "local_planning_manager/core/transition_handlers/stopping_handlers.hpp"

namespace local_planning_manager
{

class DemoAreaCStrategy : public AreaStrategy {
public:
    AreaCStrategy() {
        /*
        auto h1 = std::make_shared<PurePursuitToStoppingHandler>(0.5);
        auto h2 = std::make_shared<StoppingToPurePursuitHandler>();

        h1->set_next(h2);

        handler_chain_ = h1;
    
        */

    std::string get_area_name() const override { return "C"; }
};

} // namespace local_planning_manager
