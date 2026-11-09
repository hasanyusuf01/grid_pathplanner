# grid_pathplanner

```

ros2_ws/
└── src/
    └── grid_path_planner/
        ├── CMakeLists.txt
        ├── package.xml
        ├── include/
        │   └── grid_path_planner/
        │       ├── grid_map.hpp
        │       └── astar_planner.hpp
        ├── src/
        │   ├── grid_map.cpp
        │   ├── astar_planner.cpp
        │   └── planner_node.cpp
        ├── launch/
        │   └── planner_launch.py
        └── rviz/
            └── planner_config.rviz
```
---

## Phase 1: Workspace & Package Setup

*Goal: Initialize a clean ROS2 workspace and create the C++ package with all necessary message dependencies.*

* [ ] **Create the ROS2 Package:** Run `ros2 pkg create --build-type ament_cmake grid_path_planner --dependencies rclcpp nav_msgs geometry_msgs visualization_msgs` inside your workspace `src` folder.
* [ ] **Configure `package.xml`:** Ensure all dependencies (`rclcpp`, `nav_msgs`, `geometry_msgs`, `visualization_msgs`) are explicitly declared.
* [ ] **Configure `CMakeLists.txt`:** Set up the C++ standard (`c++17` or `c++20`), add your executable target, and link the required ROS2 libraries.
* [ ] **Set Up Directory Structure:** Create separate folders for core logic (`include/grid_path_planner/`) and node implementation (`src/`).



---
---

## Phase 2: Core Grid & Pathfinding Engine (Pure C++)

*Goal: Write the foundational data structures and the A* or Dijkstra algorithm in standalone C++ before wrapping it in ROS2.*

* [ ] **Define Cell & Grid Structures:** Create a `Cell` struct (storing coordinates `(x, y)`, `g_cost`, `h_cost`, `f_cost`, and a parent pointer for backtracking) and a `GridMap` class representing a 2D matrix (handling obstacle bounds and free space).
* [ ] **Implement Heuristic Function:** Write a Euclidean or Manhattan distance heuristic method inside your planner class.
* [ ] **Implement A* / Dijkstra Algorithm:**
* Use `std::priority_queue` with a custom comparator for the open set.
* Maintain an open set and closed set (using hash maps or 2D boolean vectors for efficient `$O(1)$` or `$O(\log N)$` lookups).
* Implement neighbor expansion (4-connected or 8-connected grid).


* [ ] **Write Unit Tests (Optional but recommended):** Test your pathfinder with a hardcoded 10x10 grid matrix in a standalone `.cpp` file using a simple terminal printout before connecting it to ROS2.
 
### Step 2.1: Design the `Cell` Struct & `GridMap` Class

*Goal: Define how individual coordinates and the overall map bounds are represented in memory.*

* [ ] **Create `include/grid_path_planner/grid_map.hpp`:**
* Define a `Cell` struct containing:
* `int x, y;` (grid coordinates)
* `double g_cost{0.0};` (cost from start to current cell)
* `double h_cost{0.0};` (heuristic estimated cost to goal)
* `double f_cost{0.0};` (`g_cost + h_cost`, used by the priority queue)
* `std::pair<int, int> parent{-1, -1};` (stores parent coordinates for easy backtracking)


* Define a `GridMap` class with private members:
* `int width_;`, `height_;`
* `std::vector<std::vector<bool>> grid_data_;` (where `true` represents an obstacle/wall and `false` is free space).


* Declare public helper methods inside `GridMap`:
* `bool isWithinBounds(int x, int y) const;`
* `bool isObstacle(int x, int y) const;`
* `void setObstacle(int x, int y, bool is_obs);`




* [ ] **Implement `src/grid_map.cpp`:**
* Write the constructor to initialize a 2D vector of size `width x height` filled with `false` (all free space).
* Implement boundary checks: return `true` only if `x >= 0 && x < width_ && y >= 0 && y < height_`.



---

### Step 2.2: Implement the Heuristic Function

*Goal: Give your A* algorithm a mathematical compass to guide it toward the goal.*

* [ ] **Choose and write the heuristic formula inside your planner class (`include/grid_path_planner/astar_planner.hpp`):**
* If using a **4-connected grid** (up, down, left, right), use **Manhattan Distance**:
`std::abs(x1 - x2) + std::abs(y1 - y2)`
* If using an **8-connected grid** (allowing diagonals), use **Euclidean Distance**:
`std::hypot(x1 - x2, y1 - y2)` or `std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2))`



---

### Step 2.3: Build the A* Priority Queue & Custom Comparator

*Goal: Ensure the algorithm always explores the node with the lowest `f_cost` next.*

* [ ] **Understand the C++ Max-Heap Pitfall:**
* `std::priority_queue` in C++ is a **max-heap** by default (it pops the largest element). For A*, you need a **min-heap** (pop the smallest `f_cost`).


* [ ] **Write a custom comparator struct:**
```cpp
struct NodeCompare {
    bool operator()(const Cell& a, const Cell& b) {
        return a.f_cost > b.f_cost; // Notice the '>' creates a min-heap
    }
};

```


* [ ] **Set up your data structures in `AStarPlanner`:**
* Open Set: `std::priority_queue<Cell, std::vector<Cell>, NodeCompare> open_set;`
* Closed Set: A 2D boolean tracking grid `std::vector<std::vector<bool>> closed_set(width, std::vector<bool>(height, false));`
* Cost-to-come map: A 2D grid `g_costs` initialized to infinity (`std::numeric_limits<double>::infinity()`) to keep track of the shortest known path to every cell.



---

### Step 2.4: Implement the Main A* Search Loop

*Goal: Write the core algorithmic loop that expands neighbors, calculates costs, and detects the goal.*

* [ ] **Define the neighbor offsets:**
* For a 4-connected grid: `std::vector<std::pair<int, int>> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};`


* [ ] **Write the search logic flow:**
1. Push the start cell into the `open_set` with `g_cost = 0` and `f_cost = heuristic(start, goal)`.
2. While `open_set` is not empty:
* Pop the node with the lowest `f_cost` (`current = open_set.top();`).
* If `current` matches the goal coordinates, break and reconstruct the path by walking backward using parent coordinates.
* Mark `current` as visited in your `closed_set`.
* Loop through all valid neighbors:
* If a neighbor is out of bounds, an obstacle, or already in the `closed_set`, skip it.
* Calculate tentative `g_cost` (`current.g_cost + movement_cost` where movement cost is `1.0` for orthogonal moves).
* If this new `g_cost` is lower than the neighbor's previously recorded `g_cost`:
* Update its parent pointers, set new `g`, `h`, and `f` costs, and push it into the `open_set`.






3. If the `open_set` empties without reaching the goal, return an empty path (no route found).



---

### Step 2.5: Write a Standalone Unit Test (`main.cpp`)

*Goal: Verify your C++ engine works perfectly in the terminal before writing any ROS2 wrapper code.*

* [ ] **Create a temporary `src/test_main.cpp` file:**
* Initialize a `GridMap` of size `10 x 10`.
* Manually add a vertical wall of obstacles from `(5, 2)` to `(5, 7)`.
* Instantiate your `AStarPlanner` with start `(2, 5)` and goal `(8, 5)`.
* Run the planner and print the resulting vector of coordinates to the terminal using `std::cout`.
* Compile and run it using a basic g++ command or a simple local CMake setup to confirm your logic has zero memory leaks, infinite loops, or segmentation faults.



---

## Phase 3: ROS2 Node Wrapper & Service Interface

*Goal: Wrap your C++ pathfinder inside an `rclcpp::Node` and expose a service to trigger planning dynamically.*

* [ ] **Create the PlannerNode Class:** Inherit from `rclcpp::Node` and initialize parameters (grid size, resolution, start/goal coordinates).
* [ ] **Define a Custom Service or Standard Interface:** Create a ROS2 service (or use `nav_msgs/srv/GetMap` / custom service) to accept a start point `(x, y)` and a goal point `(x, y)` dynamically from the terminal.
* [ ] **Integrate the Engine:** Inside the service callback, pass the start/goal coordinates to your C++ `GridMap` and pathfinder instance.
* [ ] **Handle Edge Cases:** Gracefully handle scenarios where the goal is inside an obstacle, out of bounds, or no valid path exists, returning a clear status flag.

---

## Phase 4: RViz2 Visualization Integration

*Goal: Publish `nav_msgs/msg/Path` and `visualization_msgs/msg/Marker` arrays so you can visually inspect your algorithm in real-time.*

* [ ] **Implement Path Publisher:** Convert your C++ vector of grid coordinates into a standard `nav_msgs/msg/Path` message, setting appropriate frame IDs (`map`) and timestamps, and publish it.
* [ ] **Implement Search Exploration Markers:** Create a `visualization_msgs/msg/Marker` (type `POINTS` or `CUBES_LIST`) to publish cells added to the open/closed sets during the search, creating a real-time "search expansion" animation effect.
* [ ] **Configure RViz2 Setup:** Create a configuration file or manually add a `Path` display and a `Marker` display in RViz2 mapped to your node's topics (`/planned_path` and `/search_nodes`).

---

## Phase 5: Testing, Tuning, and Launch

*Goal: Run the full stack, visualize the results, and optimize performance.*

* [ ] **Create a Launch File:** Write a Python launch file (`planner_launch.py`) to start your custom C++ node and RViz2 simultaneously with a pre-saved configuration.
* [ ] **End-to-End Test:** Use `ros2 service call` from the terminal to send various start and goal coordinates, and observe the path and search nodes update instantly in RViz2.
* [ ] **Performance Profiling:** Check CPU usage and execution time of your pathfinder for larger grids (e.g., 100x100) to ensure your C++ memory management and priority queue usage are efficient.