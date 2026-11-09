#pragma once
#include <vector>
#include <utility> 

// using namespace std;
namespace grid_path_planner{
        class GridMap{
            private:
                int width_;
                int height_;
                std::vector<std::vector<bool>> grid_data_;

            public:
                GridMap(int width, int height);
                bool isWithinBounds(int x, int y) const;
                bool isObstacle(int x, int y) const;
                void setObstacle(int x, int y, bool is_obs);
        };

        struct Cell{
            int x, y;
            double g_cost{0.0};
            double h_cost{0.0};
            double f_cost{0.0};
            std::pair<int, int> parent{-1, -1};
        };

}