```
local_planning_manager/
├── include/
│   └── local_planning_manager/
│       ├── local_planning_manager_node.hpp
│       ├── local_planning_manager_component.hpp
│       ├── transition_context.hpp
│       ├── transition_handler.hpp
│       ├── transition_handlers/
│       │   ├── pure_pursuit_handlers.hpp
│       │   ├── dwa_handlers.hpp
│       │   ├── stopping_handlers.hpp
│       │   └── inplace_turn_handlers.hpp
│       ├── area_strategy.hpp
│       └── area_strategies/
│           ├── area_a_strategy.hpp
│           ├── area_b_strategy.hpp
│           └── area_c_strategy.hpp
├── src/
│   ├── local_planning_manager.cpp
│   ├── local_planning_manager_node.cpp
│   └── local_planning_manager_component.cpp
├── test/
│   ├── test_data/
│   │   ├── process_transitions.csv
│   │   └── test_results_process_transitions.csv
│   ├── test_local_planning_manager_component.cpp
│   └── test_utils.hpp
├── config/
│   └── params.yaml
├── launch/
│   └── local_planning_manager_launch.py
├── CMakeLists.txt
└── package.xml
```

# lifecycle_manager_node
# lifecycle_manager_node
