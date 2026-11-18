#include <rclcpp/rclcpp.hpp>

#include "grid_map.hpp"
#include "astar_planner.hpp"
#include "grid_path_planner/planner_node.hpp"


path_planner::PlannerNode::PlannerNode()
    rclcpp::Node("astar_planner_node"){

    this->declare_parameter<int>("grid_width", 10);
    this->declare_parameter<int>("grid_height", 10);

    RCLCPP_INFO(this->get_logger(), "PlannerNode has been successfully initialized!");

    this->grid_map_ = grid_path_planner::GridMap(this->get_parameter("grid_width").as_int(), this->get_parameter("grid_height").as_int());


};


