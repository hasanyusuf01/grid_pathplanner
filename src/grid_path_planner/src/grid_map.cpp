#include "grid_path_planner/grid_map.hpp"

#include <iostream>

grid_path_planner::GridMap::GridMap(int height, int width)
    : width_(width), height_(height), grid_data_(height, std::vector<bool>(width, false)) {
    // The member initializer list sets width_ and height_.
    // grid_data_ is initialized as a 2D vector with 'height' rows and 'width' columns,
    // where every cell starts as 'false' (free space).
}

bool grid_path_planner::GridMap::isWithinBounds(int x, int y) const {
    if (x >= 0 && x < width_ && y >= 0 && y < height_) {
        return true;
    } else {
        return false;
    }
}

int grid_path_planner::GridMap::getWidth() const {
    return width_;
}

int grid_path_planner::GridMap::getHeight() const {
    return height_;
}


