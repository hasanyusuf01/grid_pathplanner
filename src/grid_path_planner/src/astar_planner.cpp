#include <cmath>
# include "astar_planner.hpp"
# include "grid_map.hpp"

astar_planner::AStarPlanner::AStarPlanner(const grid_path_planner::Cell& goal_cell) : goal_(goal_cell){} 
double astar_planner::AStarPlanner::manhatanDist(int current_x, int current_y) const{
                double h_cost;
                h_cost = std::abs(current_x - astar_planner::AStarPlanner::goal_.x)+ std::abs(current_y - astar_planner::AStarPlanner::goal_.y);
                return h_cost;
            };

double astar_planner::AStarPlanner::eucDist(int current_x, int current_y) const{
                double h_cost;
                h_cost = std::hypot(std::abs(current_x - goal_.x),std::abs(current_y - goal_.y));
                return h_cost;
            };

std::vector<grid_path_planner::Cell> astar_planner::AStarPlanner::planPath(const grid_path_planner::GridMap& grid_map, const grid_path_planner::Cell& start_cell){
    std::priority_queue<grid_path_planner::Cell, std::vector<grid_path_planner::Cell>, NodeCompare> open_set;
    // std::vector<std::vector<bool>> closed_set(width, std::vector<bool>(height, false));
    std::vector<grid_path_planner::Cell> path;
    return path;
};
