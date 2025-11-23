#include "local_planning_manager/local_planning_manager_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<local_planning_manager::LocalPlanningManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
