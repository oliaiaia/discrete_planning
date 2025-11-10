#include "AStar_interface.hpp"


// Python list -> vector<vector<int>>
std::vector<std::vector<int>> py_list_to_vector2d_int(const py::list &list)
{
    std::vector<std::vector<int>> result;
    for (size_t i = 0; i < list.size(); ++i)
    {
        py::list row = list[i].cast<py::list>();
        std::vector<int> row_vec;
        for (size_t j = 0; j < row.size(); ++j)
        {
            row_vec.push_back(row[j].cast<int>());
        }
        result.push_back(row_vec);
    }
    return result;
}

// Python list -> vector<vector<vector<int>>>
std::vector<std::vector<std::vector<int>>> py_list_to_vector3d_int(const py::list &list)
{
    std::vector<std::vector<std::vector<int>>> result;
    for (size_t i = 0; i < list.size(); ++i)
    {
        py::list config = list[i].cast<py::list>();
        std::vector<std::vector<int>> config_vec;
        for (size_t j = 0; j < config.size(); ++j)
        {
            py::list row = config[j].cast<py::list>();
            std::vector<int> row_vec;
            for (size_t k = 0; k < row.size(); ++k)
            {
                row_vec.push_back(row[k].cast<int>());
            }
            config_vec.push_back(row_vec);
        }
        result.push_back(config_vec);
    }
    return result;
}

// vector<Position> -> Python list
py::list vector_to_py_list(const std::vector<Position> &positions)
{
    py::list result;
    for (const auto &pos : positions)
    {
        result.append(py::make_tuple(pos.x, pos.y, pos.theta));
    }
    return result;
}

// Eigen::MatrixXd -> Python list
py::list matrix_to_py_list(const Eigen::MatrixXd &matrix)
{
    py::list result;
    for (int i = 0; i < matrix.rows(); ++i)
    {
        py::list row;
        for (int j = 0; j < matrix.cols(); ++j)
        {
            row.append(matrix(i, j));
        }
        result.append(row);
    }
    return result;
}

PyAStar::PyAStar(const py::list &environment_list,
                 const py::list &rod_list,
                 const std::string &heuristic_type)
{

    auto environment = py_list_to_vector2d_int(environment_list);
    auto rod = py_list_to_vector3d_int(rod_list);

    Wrapper wrapper(rod, environment);
    cspace_mats_ = wrapper.getCSpace();

    if (heuristic_type == "manhattan")
    {
        heuristic_function_ = [](std::shared_ptr<Node> a, std::shared_ptr<Node> b)
        {
            return std::abs(a->rowNum - b->rowNum) + std::abs(a->colNum - b->colNum);
        };
        std::cout << "Using Manhattan heuristic" << std::endl;
    }
    else if (heuristic_type == "dijkstra")
    {
        heuristic_function_ = [](std::shared_ptr<Node> a, std::shared_ptr<Node> b)
        {
            return 0.0;
        };
        std::cout << "Using Dijkstra heuristic" << std::endl;
    }
    else if (heuristic_type == "orientation")
    {
        heuristic_function_ = [](std::shared_ptr<Node> a, std::shared_ptr<Node> b)
        {
            double coeff = 10;
            return std::abs(a->rowNum - b->rowNum) + std::abs(a->colNum - b->colNum) + coeff * std::abs(a->theta - b->theta);
        };
        std::cout << "Using angle depend heuristic" << std::endl;
    }
    else
    {
        throw std::invalid_argument("Unknown heuristic type: " + heuristic_type);
    }
}

py::list PyAStar::find_path(int start_x, int start_y, int start_theta,
                            int goal_x, int goal_y, int goal_theta)
{

    std::cout << "Starting path finding..." << std::endl;

    AStar astar(start_x, start_y, start_theta, goal_x, goal_y, goal_theta,
                cspace_mats_, heuristic_function_);

    auto plan = astar.getPlan();

    std::cout << "Path finding completed. Path length: " << plan.size() << std::endl;

    return vector_to_py_list(plan);
}

py::list PyAStar::get_cspace() const
{
    py::list result;
    for (const auto &matrix : cspace_mats_)
    {
        result.append(matrix_to_py_list(matrix));
    }
    return result;
}

py::list find_path_direct(
    const py::list &environment_list,
    const py::list &rod_list,
    int start_x, int start_y, int start_theta,
    int goal_x, int goal_y, int goal_theta,
    const std::string &heuristic_type)
{

    PyAStar planner(environment_list, rod_list, heuristic_type);
    return planner.find_path(start_x, start_y, start_theta, goal_x, goal_y, goal_theta);
}

py::list compute_cspace_direct(
    const py::list &environment_list,
    const py::list &rod_list)
{

    auto environment = py_list_to_vector2d_int(environment_list);
    auto rod = py_list_to_vector3d_int(rod_list);

    Wrapper wrapper(rod, environment);
    auto cspace_mats = wrapper.getCSpace();

    py::list result;
    for (const auto &matrix : cspace_mats)
    {
        result.append(matrix_to_py_list(matrix));
    }
    return result;
}

PYBIND11_MODULE(astar_planner, m)
{
    m.doc() = "A* path planning algorithm for discrete environments with rod configurations";

    py::class_<PyAStar>(m, "AStar")
        .def(py::init<const py::list &, const py::list &, const std::string &>(),
             py::arg("environment"),
             py::arg("rod"),
             py::arg("heuristic_type"),
             "Initialize A* planner with environment and rod configuration")
        .def("find_path", &PyAStar::find_path,
             py::arg("start_x"), py::arg("start_y"), py::arg("start_theta"),
             py::arg("goal_x"), py::arg("goal_y"), py::arg("goal_theta"),
             "Find path from start to goal position")
        .def("get_cspace", &PyAStar::get_cspace,
             "Get the C-Space matrices for all orientations");

    m.def("find_path", &find_path_direct,
          py::arg("environment"), py::arg("rod"),
          py::arg("start_x"), py::arg("start_y"), py::arg("start_theta"),
          py::arg("goal_x"), py::arg("goal_y"), py::arg("goal_theta"),
          py::arg("heuristic_type"),
          "Find path directly without creating planner object");

    m.def("compute_cspace", &compute_cspace_direct,
          py::arg("environment"), py::arg("rod"),
          "Compute C-Space matrices directly without creating planner object");

    m.attr("HEURISTIC_MANHATTAN") = py::str("manhattan");
    m.attr("HEURISTIC_DIJKSTRA") = py::str("dijkstra");
}