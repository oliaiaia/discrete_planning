#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <vector>
#include "AStar_planner.hpp"
#include "Wrapper.hpp"

namespace py = pybind11;

class PyAStar {
public:
    PyAStar(const py::list& environment_list,
            const py::list& rod_list,
            const std::string& heuristic_type);
    
    py::list find_path(int start_x, int start_y, int start_theta,
                      int goal_x, int goal_y, int goal_theta);
    
    py::list get_cspace() const;
    
private:
    std::vector<Eigen::MatrixXd> cspace_mats_;
    HeuristicFunction heuristic_function_;
};

std::vector<std::vector<int>> py_list_to_vector2d_int(const py::list& list);
std::vector<std::vector<std::vector<int>>> py_list_to_vector3d_int(const py::list& list);
py::list vector_to_py_list(const std::vector<Position>& positions);
py::list matrix_to_py_list(const Eigen::MatrixXd& matrix);