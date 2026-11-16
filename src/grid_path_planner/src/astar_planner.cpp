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

std::vector<grid_path_planner::Cell> astar_planner::AStarPlanner::planPath(
    const grid_path_planner::GridMap& grid_map, 
    grid_path_planner::Cell& goal_cell, 
    grid_path_planner::Cell& start_cell)
    
    
    
    {


        int width = grid_map.getWidth();
        int height = grid_map.getHeight();

        //  tracking data structures: 

        // open set
        std::priority_queue<grid_path_planner::Cell, std::vector<grid_path_planner::Cell>, NodeCompare> open_set;
        //  closed set : for marking visited nodes
        std::vector<std::vector<bool>> closed_set(height, std::vector<bool>(width, false));
        // for tracking the g_cost
        std::vector<std::vector<double>> g_costs(height, std::vector<double>(width, std::numeric_limits<double>::infinity()));
        std::vector<grid_path_planner::Cell> path;
        // Tracks the (x, y) coordinates of the parent for every cell on the grid
        std::vector<std::vector<std::pair<int, int>>> parent_map(
            height, std::vector<std::pair<int, int>>(width, {-1, -1})
        );

        // //  assumptions : let the start cell be 0 , 0  /==/  goal cell for now be 
        // // g_cost of the start cell == 0.0
        int start_x = start_cell.x;
        int start_y = start_cell.y;

        // start_x , start_y, start_cell.y, start_cell.x = 0 ;
        start_cell.g_cost = 0.0;
        start_cell.h_cost = manhatanDist(0,0);
        start_cell.f_cost = start_cell.g_cost+start_cell.h_cost;
        start_cell.parent = {-1,-1};
        parent_map[start_y][start_x] = {-1,-1};



        // open_set.push(neighbor_cell); Cell current = open_set.top(); open_set.pop();
        g_costs[start_y][start_x] =0.0;
        while(!open_set.empty())
        { 
            // node with lowest fcost
            grid_path_planner::Cell current_node = open_set.top();
            if (current_node.x != goal_cell.x && current_node.y != goal_cell.y){
                int g_cost_curr = current_node.g_cost;
                closed_set[current_node.x][current_node.y]= true;
                //  neighbours 
                    // up
                     grid_path_planner::Cell  N_up =  {current_node.x , current_node.y-1};
                    // down
                     grid_path_planner::Cell   N_dwn =  {current_node.x , current_node.y+1};
                    // right
                     grid_path_planner::Cell   N_rht =  {current_node.x-1 , current_node.y};
                    // left
                     grid_path_planner::Cell   N_lft =  {current_node.x+1, current_node.y};

                    std::vector< grid_path_planner::Cell > neighbours = {N_up, N_dwn, N_lft, N_rht};

                    for ( auto& neighbour: neighbours )
                    {

                        int x, y;
                        x = neighbour.x;
                        y = neighbour.y;
                        if(grid_map.isWithinBounds(x,y))
                        {
                            if(grid_map.isObstacle(x,y))
                            {
                               continue;
                            }
                            else
                            {
                                 if(closed_set[x][y]  == true)
                                {// already visited neighbour skip
                                    continue;
                                }
                                else{
                                    // not visited neighbour 
                                    neighbour.g_cost = g_cost_curr + 1;
                                    if (neighbour.g_cost < g_costs[x][y]){
                                        g_costs[x][y] = neighbour.g_cost; // update the g cost of the neighbour
                                    };
                                    neighbour.h_cost = manhatanDist(x,y);
                                    neighbour.f_cost = neighbour.g_cost+neighbour.h_cost;
                                    neighbour.parent = {current_node.x,current_node.y};
                                    parent_map[y][x] = {current_node.x, current_node.y};
                                    open_set.push(neighbour);
                                }
                            }
                        }
                        else
                        {
                            // skipp outof bounds neighbours
                            continue;
                        }
                    }
            }
            //  if current == goal;
            else{
                break;// std::cout<<"Goal Reached";
                }

        }// loop break 

        auto curr_node = goal_cell;
        // std::vector<

        while(curr_node.parent[0] != -1){
            path.push_back(curr_node); 

            curr_node = 
        }


        



        return path;
};