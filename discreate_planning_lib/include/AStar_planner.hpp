#pragma once

#include <stdexcept>
#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
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

struct CSpace
{
    Eigen::MatrixXd cSpaceMat;
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> visitedFlag;

    CSpace(const Eigen::MatrixXd &cSpaceMat) : cSpaceMat(cSpaceMat)
    {
        visitedFlag.resize(cSpaceMat.rows(), cSpaceMat.cols());
        visitedFlag.setConstant(false); // заполняем false
    }
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

    bool operator>(const Node &other) const
    {
        return (heuristic > other.heuristic);
    }
    bool operator<(const Node &other) const
    {
        return (heuristic < other.heuristic);
    }
};


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
    std::vector<CSpace> cSpaceFull;
    std::priority_queue<std::shared_ptr<Node>, 
                    std::vector<std::shared_ptr<Node>>,
                    std::function<bool(std::shared_ptr<Node>, std::shared_ptr<Node>)>> Q;

    std::shared_ptr<Node> startNode;
    std::shared_ptr<Node> endNode;
    HeuristicFunction heuristicFunction;

    int steps = 0;
    double finalSum;

    double manhattanDistance(std::shared_ptr<Node> node1, std::shared_ptr<Node> node2) const;
    // in our case always = 1
    double eulerDist(const Node& node1, const Node& node2) const;

    void getFeasibleNodes(std::shared_ptr<Node> currentNode, std::vector<std::shared_ptr<Node>> &feasibleNodes);
    // int getSteps();

    void launchAStart();

    bool isValidPosition(int x, int y, int theta) const;
};
