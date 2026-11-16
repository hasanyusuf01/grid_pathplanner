#pragma once
#include <cmath>


#include "grid_map.hpp"
#include <queue>
#include <vector>

struct NodeCompare {
    bool operator()(const grid_path_planner::Cell& a, const grid_path_planner::Cell& b) {
        return a.f_cost > b.f_cost; // Notice the '>' creates a min-heap
    }
};


namespace astar_planner {


    class AStarPlanner {
        private:
            // Private member variables and methods for the A* algorithm
            grid_path_planner::Cell goal_;

            // hurestics 
            // The heuristic function (h_cost) calculates 
            // a guess of how much distance/cost is left between 
            // the current cell and the goal cell.
            double manhatanDist(int current_x, int current_y) const;
            double  eucDist(int current_x, int current_y)const;
        public:
                AStarPlanner(const grid_path_planner::Cell& goal_cell);
                std::vector<grid_path_planner::Cell> planPath(const grid_path_planner::GridMap& grid_map,grid_path_planner::Cell& goal_cell, grid_path_planner::Cell& start_cell);


    };
}