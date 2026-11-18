#pragma once
#include <rclcpp/rclcpp.hpp>

#include "grid_map.hpp"
#include "astar_planner.hpp"

namespace path_planner {
            class PlannerNode : public rclcpp::Node {
                private:
                grid_path_planner::GridMap grid_map_;
                astar_planner::AStarPlanner planner_;

                public:
                PlannerNode();



            };
}