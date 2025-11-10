#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "AStar_planner.hpp"
#include "Wrapper.hpp"

void testSmallEnvironment() {
    std::cout << "=== TEST 1: Small Environment ===" << std::endl;
    
    // Создаем маленькое окружение 5x5
    std::vector<std::vector<int>> environment = {
        {0, 0, 0, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 1, 0, 1, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 0, 0, 0}
    };
    
    // Простой стержень 1x1
    std::vector<std::vector<std::vector<int>>> rod = {
        {{{1}}}
    };
    
    std::cout << "Environment size: " << environment.size() << "x" << environment[0].size() << std::endl;
    std::cout << "Rod configurations: " << rod.size() << std::endl;
    
    // Создаем Wrapper и вычисляем C-Space
    Wrapper wrapper(rod, environment);
    auto cspaceMats = wrapper.getCSpace();
    
    std::cout << "C-Space matrices: " << cspaceMats.size() << std::endl;
    std::cout << "C-Space matrices: " << cspaceMats[0] << std::endl;
    if (!cspaceMats.empty()) {
        std::cout << "C-Space matrix size: " << cspaceMats[0].rows() << "x" << cspaceMats[0].cols() << std::endl;
    }
    
    // Тестируем A* с разными точками
    std::cout << "\n--- Testing A* from (1,1) to (3,3) ---" << std::endl;
    AStar astar1(1, 1, 0, 3, 3, 0, cspaceMats);
    auto plan1 = astar1.getPlan();
    std::cout << "Path length: " << plan1.size() << std::endl;
    for (const auto& pos : plan1) {
        std::cout << "  (" << pos.x << ", " << pos.y << ", " << pos.theta << ")" << std::endl;
    }
    
    std::cout << "\n--- Testing A* from (0,0) to (4,4) ---" << std::endl;
    AStar astar2(0, 0, 0, 4, 4, 0, cspaceMats);
    auto plan2 = astar2.getPlan();
    std::cout << "Path length: " << plan2.size() << std::endl;
    for (const auto& pos : plan2) {
        std::cout << "  (" << pos.x << ", " << pos.y << ", " << pos.theta << ")" << std::endl;
    }
}

void testMediumEnvironment() {
    std::cout << "\n=== TEST 2: Medium Environment ===" << std::endl;
    
    // Окружение 10x10 с коридором
    std::vector<std::vector<int>> environment = {
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 1, 1, 1, 1, 1, 1, 0, 1},
        {1, 0, 1, 0, 0, 0, 0, 1, 0, 1},
        {1, 0, 1, 0, 1, 1, 0, 1, 0, 1},
        {1, 0, 1, 0, 1, 1, 0, 1, 0, 1},
        {1, 0, 1, 0, 0, 0, 0, 1, 0, 1},
        {1, 0, 1, 1, 1, 1, 1, 1, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
    };
    
    // Стержень 2x2
    std::vector<std::vector<std::vector<int>>> rod = {
        {
            {1, 0},
            {0, 1}
        },
        {
            {0, 1},
            {1, 0}
        }
    };
    
    std::cout << "Environment size: " << environment.size() << "x" << environment[0].size() << std::endl;
    std::cout << "Rod configurations: " << rod.size() << std::endl;
    
    Wrapper wrapper(rod, environment);
    auto cspaceMats = wrapper.getCSpace();
    
    std::cout << "C-Space matrices: " << cspaceMats.size() << std::endl;
    
    // Тестируем разные ориентации
    std::cout << "\n--- Testing A* with orientation changes ---" << std::endl;
    AStar astar(1, 1, 0, 8, 8, 1, cspaceMats);
    auto plan = astar.getPlan();
    std::cout << "Path length: " << plan.size() << std::endl;
    for (const auto& pos : plan) {
        std::cout << "  (" << pos.x << ", " << pos.y << ", " << pos.theta << ")" << std::endl;
    }
}

void testCSpaceComputation() {
    std::cout << "\n=== TEST 3: C-Space Computation ===" << std::endl;
    
    // Простое окружение для проверки свертки
    std::vector<std::vector<int>> environment = {
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0}
    };
    
    // Стержень 3x3
    std::vector<std::vector<std::vector<int>>> rod = {
        {
            {1, 1, 1},
            {1, 1, 1},
            {1, 1, 1}
        }
    };
    
    Wrapper wrapper(rod, environment);
    auto cspaceMats = wrapper.getCSpace();
    
    if (!cspaceMats.empty()) {
        std::cout << "C-Space matrix (should show obstacle expanded):" << std::endl;
        Eigen::MatrixXd cspace = cspaceMats[0];
        
        for (int i = 0; i < cspace.rows(); ++i) {
            for (int j = 0; j < cspace.cols(); ++j) {
                std::cout << (cspace(i, j) > 0.5 ? "1 " : "0 ");
            }
            std::cout << std::endl;
        }
    }
}

void testNodeComparison() {
    std::cout << "\n=== TEST 4: Node Comparison Operator ===" << std::endl;
    
    // Проверяем оператор сравнения
    Node node1(1, 1, 0);
    Node node2(1, 1, 0);
    Node node3(1, 2, 0);
    Node node4(1, 1, 1);
    
    std::cout << "node1 == node2: " << (node1 == node2 ? "true" : "false") << " (should be true)" << std::endl;
    std::cout << "node1 == node3: " << (node1 == node3 ? "true" : "false") << " (should be false)" << std::endl;
    std::cout << "node1 == node4: " << (node1 == node4 ? "true" : "false") << " (should be false)" << std::endl;
    
    // Проверяем разные координаты но одинаковые точки
    Node startNode(6, 6, 2);
    Node endNode(85, 85, 0);
    std::cout << "startNode == endNode: " << (startNode == endNode ? "true" : "false") << " (should be false)" << std::endl;
}

void testLargeEnvironment() {
    std::cout << "\n=== TEST 5: Large Environment (Your Case) ===" << std::endl;
    
    // Создаем большое окружение 100x100
    std::vector<std::vector<int>> environment(100, std::vector<int>(100, 0));
    
    // Добавляем несколько препятствий
    for (int i = 30; i < 70; ++i) {
        for (int j = 30; j < 70; ++j) {
            environment[i][j] = 1;
        }
    }
    
    // Простой стержень
    std::vector<std::vector<std::vector<int>>> rod = {
        {{{1}}},
        {{{1}}},
        {{{1}}}
    };
    
    std::cout << "Environment size: " << environment.size() << "x" << environment[0].size() << std::endl;
    
    Wrapper wrapper(rod, environment);
    auto cspaceMats = wrapper.getCSpace();
    
    std::cout << "C-Space matrices: " << cspaceMats.size() << std::endl;
    
    // Тестируем ваши конкретные точки
    std::cout << "\n--- Testing your specific case: (6,6,2) to (85,85,0) ---" << std::endl;
    AStar astar(6, 6, 2, 85, 85, 0, cspaceMats);
    auto plan = astar.getPlan();
    std::cout << "Path length: " << plan.size() << std::endl;
    
    if (plan.empty()) {
        std::cout << "NO PATH FOUND!" << std::endl;
    } else {
        std::cout << "First 5 points:" << std::endl;
        for (size_t i = 0; i < std::min(plan.size(), size_t(5)); ++i) {
            std::cout << "  (" << plan[i].x << ", " << plan[i].y << ", " << plan[i].theta << ")" << std::endl;
        }
        if (plan.size() > 5) {
            std::cout << "  ... and " << (plan.size() - 5) << " more points" << std::endl;
        }
    }
}

void testObstacleEnvironment() {
    std::cout << "\n=== TEST 6: Environment with Obstacles ===" << std::endl;
    
    std::vector<std::vector<int>> environment = {
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 1, 1, 1, 1, 1, 1, 0, 0},
        {0, 0, 1, 0, 0, 0, 0, 1, 0, 0},
        {0, 0, 1, 0, 0, 0, 0, 1, 0, 0},
        {0, 0, 1, 0, 0, 0, 0, 1, 0, 0},
        {0, 0, 1, 0, 0, 0, 0, 1, 0, 0},
        {0, 0, 1, 1, 1, 1, 1, 1, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    };
    
    std::vector<std::vector<std::vector<int>>> rod = {
        {{{1}}}
    };
    
    Wrapper wrapper(rod, environment);
    auto cspaceMats = wrapper.getCSpace();
    
    // Пытаемся найти путь через препятствие (должен обойти)
    std::cout << "Testing path around obstacle: (1,1) to (8,8)" << std::endl;
    AStar astar(1, 1, 0, 8, 8, 0, cspaceMats);
    auto plan = astar.getPlan();
    std::cout << "Path length: " << plan.size() << std::endl;
    
    // Проверяем, что путь не проходит через препятствия
    if (!plan.empty()) {
        bool valid = true;
        for (const auto& pos : plan) {
            if (environment[pos.y][pos.x] == 1) {
                std::cout << "ERROR: Path goes through obstacle at (" << pos.x << ", " << pos.y << ")" << std::endl;
                valid = false;
            }
        }
        if (valid) {
            std::cout << "Path is valid - no obstacles encountered" << std::endl;
        }
    }
}

int main() {
    std::cout << "A* Algorithm Test Suite" << std::endl;
    std::cout << "========================" << std::endl;
    
    // testNodeComparison();  // Сначала проверяем оператор сравнения
    
    testSmallEnvironment();
    // testMediumEnvironment(); 
    testCSpaceComputation();
    // testLargeEnvironment();
    testObstacleEnvironment();
    
    std::cout << "\n=== ALL TESTS COMPLETED ===" << std::endl;
    
    return 0;
}