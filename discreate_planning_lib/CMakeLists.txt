cmake_minimum_required(VERSION 3.10)

project(discreate_planning_lib)
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(${CMAKE_SOURCE_DIR}/include)

find_package(Eigen3 3.3 REQUIRED)
add_executable(discreate_planning src/AStar_planner.cpp src/Wrapper.cpp src/main.cpp)
target_link_libraries(discreate_planning Eigen3::Eigen)


find_package(Python 3.6 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

pybind11_add_module(astar_planner
    src/AStar_interface.cpp
    src/AStar_planner.cpp
    src/Wrapper.cpp
)

target_include_directories(astar_planner PRIVATE include)
target_link_libraries(astar_planner PRIVATE Eigen3::Eigen)