#pragma once

#include <stdexcept>
#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <queue>
#include <memory>
#include <functional>

struct Position
{
    int x;
    int y;
    int theta;
};


struct Node
{
    int rowNum, colNum, theta; // rowNum - coloumn, colNum - row
    double lengthFromStart = 0.0;
    double heuristic = 0.0;

    std::shared_ptr<Node> prevNode = nullptr;

    Node() = default;
    Node(int rowNum, int colNum, int theta) : rowNum(rowNum), colNum(colNum), theta(theta) {}
    Node(int rowNum, int colNum, int theta, Node *prevNode) : rowNum(rowNum), colNum(colNum), theta(theta), prevNode(prevNode) {}

    bool operator==(const Node &other) const
    {
        return (rowNum == other.rowNum) &&
            (colNum == other.colNum) &&
            (theta == other.theta);
    }

    bool operator>(const Node &other) const {return (heuristic > other.heuristic);}
    bool operator<(const Node &other) const { return (heuristic < other.heuristic);}

};

namespace std
{
template <>
struct hash<Node>
{
    std::size_t operator()(const Node &n) const noexcept
    {
        std::size_t h1 = std::hash<int>()(n.rowNum);
        std::size_t h2 = std::hash<int>()(n.colNum);
        std::size_t h3 = std::hash<int>()(n.theta);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};
}

/**
 * @brief find path for point from StartPos to EndPos in CSpace
 * @param StartPos, EndPos, CSpace
 */

using HeuristicFunction = std::function<double(std::shared_ptr<Node>, std::shared_ptr<Node>)>;

class AStar
{

public:
    AStar(int startPosX, int startPosY, int startTetha, int endPosX, int endPosY, int endTetha, 
        const std::vector<Eigen::MatrixXd> &cSpaceMats, HeuristicFunction heuristicFunc = nullptr);
    std::vector<Position> getPlan();

private:
    std::vector<Eigen::MatrixXd> cSpace;
    std::priority_queue<std::shared_ptr<Node>, 
                    std::vector<std::shared_ptr<Node>>,
                    std::function<bool(std::shared_ptr<Node>, std::shared_ptr<Node>)>> Q;

    std::shared_ptr<Node> startNode;
    std::shared_ptr<Node> endNode;
    HeuristicFunction heuristicFunction;

    std::unordered_map<Node, std::shared_ptr<Node>> allNodesMap;

    int steps = 0;

    double manhattanDistance(std::shared_ptr<Node> node1, std::shared_ptr<Node> node2) const;

    void getFeasibleNodes(std::shared_ptr<Node> currentNode, std::vector<std::shared_ptr<Node>> &feasibleNodes);

    void launchAStart();

    bool isValidPosition(int x, int y, int theta) const;
};
