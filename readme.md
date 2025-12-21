# AStar Path Planning Library in discreate space

## Prerequisites

- C++17 compatible compiler
- Python 3.6+ with development headers
- Eigen3 3.3 or higher
- PyBind11


## Project Structure

```
your_project/
├── main.ipynb
├── astar_planner.cpython-312-x86_64-linux-gnu.so  # ← Place here after build
├── discreate_planning_lib/
|   ├── CMakeLists.txt
|   ├──include/
│       ├── AStar_planner.hpp
│       ├── AStar_interface.hpp
│       └── Wrapper.hpp
|   ├── src/
│       ├── AStar_interface.cpp
│       ├── AStar_planner.cpp
│       └── Wrapper.cpp
└   ├──  build/                    # ← Build directory
```

The library provides path planning for 2d-object with orientation constraints.

## Result

![gif](rod_solve.gif)

