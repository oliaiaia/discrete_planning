#include "AStar_planner.hpp"


AStar::AStar(int startPosX, int startPosY, int startTetha, int endPosX, int endPosY, 
    int endTetha, const std::vector<Eigen::MatrixXd> &cSpaceMats, HeuristicFunction heuristicFunc)
 : Q([](std::shared_ptr<Node> a, std::shared_ptr<Node> b) { return a->heuristic > b->heuristic; }), heuristicFunction(heuristicFunc)
{

    if (!heuristicFunction) {
        heuristicFunction = [this](std::shared_ptr<Node> node1, std::shared_ptr<Node> node2) {
            return this->manhattanDistance(node1, node2);
        };
    }

    for (const auto &cSpaceMat : cSpaceMats)
    {
        CSpace cSpace(cSpaceMat);
        cSpaceFull.push_back(cSpace);
    }

    startNode = std::make_shared<Node>(startPosY, startPosX, startTetha);
    endNode = std::make_shared<Node>(endPosY, endPosX, endTetha);

    startNode->lengthFromStart = 0;
    startNode->heuristic = heuristicFunction(startNode, endNode);



    if (!isValidPosition(startNode->rowNum, startNode->colNum, startNode->theta)) {
        std::cout << "ERROR: Start position is invalid or out of bounds!" << std::endl;
        return;
    }
    
    if (!isValidPosition(endNode->rowNum, endNode->colNum, endNode->theta)) {
        std::cout << "ERROR: End position is invalid or out of bounds!" << std::endl;
        return;
    }

    if (*startNode == *endNode)
    {
        endNode->prevNode = startNode;
        std::cout << "Start node is equal to end node. Finished the path." << std::endl;
        return;
    }

    Q.push(startNode);

    launchAStart();
}

std::vector<Position> AStar::getPlan()
{
    std::vector<Position> plan;
    if (endNode->prevNode == nullptr)
    {
        std::cout << "No path found!" << std::endl;
        return plan;
    }

    std::shared_ptr<Node> currentNode = endNode;
    while (currentNode != nullptr)
    {
        Position pos;
        pos.x = currentNode->colNum;
        pos.y = currentNode->rowNum;
        pos.theta = currentNode->theta;
        plan.push_back(pos);

        currentNode = currentNode->prevNode;
    }

    std::reverse(plan.begin(), plan.end());
    return plan;
}

double AStar::manhattanDistance(std::shared_ptr<Node> node1, std::shared_ptr<Node> node2) const
{
    return (abs(node1->rowNum - node2->rowNum) + abs(node1->colNum - node2->colNum));
}

// in our case always = 1
double AStar::eulerDist(const Node& node1, const Node& node2) const
{
    return std::sqrt(pow(node1.rowNum - node2.rowNum, 2) + pow(node1.colNum - node2.colNum, 2));
}

bool AStar::isValidPosition(int row, int col, int theta) const
{
    if (theta < 0 || theta >= static_cast<int>(cSpaceFull.size())) {
        return false;
    }
    if (row < 0 || row >= cSpaceFull[theta].cSpaceMat.rows() || 
        col < 0 || col >= cSpaceFull[theta].cSpaceMat.cols()) {
        return false;
    }
    return cSpaceFull[theta].cSpaceMat(row, col) == 0.0;
}

void AStar::getFeasibleNodes(std::shared_ptr<Node> currentNode, std::vector<std::shared_ptr<Node>> &feasibleNodes)
{

    const int drow[] = {0, 1, 0, -1};
    const int dcol[] = {1, 0, -1, 0};

    // new position
    for (int i = 0; i < 4; ++i) {
        int newX = currentNode->rowNum + drow[i];
        int newY = currentNode->colNum + dcol[i];
        
        if (isValidPosition(newX, newY, currentNode->theta)) {
            auto newNode = std::make_shared<Node>(newX, newY, currentNode->theta);
            feasibleNodes.push_back(newNode);
        }
    }

    // new orientation
    for (int newTheta = 0; newTheta < static_cast<int>(cSpaceFull.size()); newTheta++) {
        if (newTheta != currentNode->theta && isValidPosition(currentNode->rowNum, currentNode->colNum, newTheta)) {
            auto newNode = std::make_shared<Node>(currentNode->rowNum, currentNode->colNum, newTheta);
            feasibleNodes.push_back(newNode);
        }
    }

}

void AStar::launchAStart()
{
    while (!Q.empty())
    {
        steps++;
        auto currentNode = Q.top();
        Q.pop();

        if (*currentNode == *endNode)
        {
            endNode->prevNode = currentNode;
            std::cout << "Found the path!" << std::endl;
            return;
        }

        std::vector<std::shared_ptr<Node>> feasibleNodes;
        getFeasibleNodes(currentNode, feasibleNodes);

        for (auto &feasibleNode : feasibleNodes)
        {

            if (!cSpaceFull[feasibleNode->theta].visitedFlag(feasibleNode->rowNum, feasibleNode->colNum))
            {
                cSpaceFull[feasibleNode->theta].visitedFlag(feasibleNode->rowNum, feasibleNode->colNum) = true;

                feasibleNode->prevNode = currentNode;
                feasibleNode->lengthFromStart = currentNode->lengthFromStart + 1; // eulerDist(currentNode, feasibleNode) or 1.0
                feasibleNode->heuristic = feasibleNode->lengthFromStart + heuristicFunction(feasibleNode, endNode);
                Q.push(feasibleNode);
            }
            else
            {
                if (feasibleNode->lengthFromStart > currentNode->lengthFromStart + 1)
                {
                    feasibleNode->prevNode = currentNode;
                    feasibleNode->lengthFromStart = currentNode->lengthFromStart + 1;
                    feasibleNode->heuristic = feasibleNode->lengthFromStart + heuristicFunction(feasibleNode, endNode);
                }
            }
        }
    }
}


