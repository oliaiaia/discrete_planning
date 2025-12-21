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

    cSpace = cSpaceMats;

    startNode = std::make_shared<Node>(startPosY, startPosX, startTetha);
    startNode->lengthFromStart = 0;
    allNodesMap[*startNode] = startNode;

    endNode = std::make_shared<Node>(endPosY, endPosX, endTetha);

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
        std::cout << "Final coast is " << 0 << std::endl;
        std::cout << "Num of states is " << 0 << std::endl;
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

bool AStar::isValidPosition(int row, int col, int theta) const
{
    if (theta < 0 || theta >= static_cast<int>(cSpace.size())) {
        return false;
    }
    if (row < 0 || row >= cSpace[theta].rows() || 
        col < 0 || col >= cSpace[theta].cols()) {
        return false;
    }
    return cSpace[theta](row, col) == 0.0;
}

void AStar::getFeasibleNodes(std::shared_ptr<Node> currentNode, std::vector<std::shared_ptr<Node>> &feasibleNodes)
{

    const int drow[] = {0, 1, 0, -1};
    const int dcol[] = {1, 0, -1, 0};

    // new position
    for (int i = 0; i < 4; ++i) {
        int nextRow = currentNode->rowNum + drow[i];
        int nextCol = currentNode->colNum + dcol[i];
        
        if (isValidPosition(nextRow, nextCol, currentNode->theta)) {
            auto newNode = std::make_shared<Node>(nextRow, nextCol, currentNode->theta);
            auto [it, inserted] = allNodesMap.insert({*newNode, newNode});
            if (!inserted) {
                newNode = it->second;
            }
            feasibleNodes.push_back(newNode);
        }
    }

    // new orientation
    for (int newTheta = 0; newTheta < static_cast<int>(cSpace.size()); newTheta++) {
        if (newTheta != currentNode->theta && isValidPosition(currentNode->rowNum, currentNode->colNum, newTheta)) {
            auto newNode = std::make_shared<Node>(currentNode->rowNum, currentNode->colNum, newTheta);
            auto [it, inserted] = allNodesMap.insert({*newNode, newNode});
            if (!inserted) {
                newNode = it->second;
            }
            feasibleNodes.push_back(newNode);
        }
    }

}

void AStar::launchAStart()
{
    while (!Q.empty())
    {
        auto currentNode = Q.top();
        Q.pop();

        states++;

        if (*currentNode == *endNode)
        {
            endNode = currentNode;
            std::cout << "Found the path!" << std::endl;
            std::cout << "Final cost is " << endNode->lengthFromStart << std::endl;
            std::cout << "Num of states is " << states << std::endl;
            return;
        }

        std::vector<std::shared_ptr<Node>> feasibleNodes;
        getFeasibleNodes(currentNode, feasibleNodes);

        for (auto &feasibleNode : feasibleNodes)
        {
            double updatedLength = (feasibleNode->theta != currentNode->theta) ? currentNode->lengthFromStart + 0.5 : currentNode->lengthFromStart + 1.0;

            if (feasibleNode->lengthFromStart == 0.0 && feasibleNode != startNode)
            {
                feasibleNode->lengthFromStart = updatedLength;
                feasibleNode->prevNode = currentNode;
                feasibleNode->heuristic = updatedLength + heuristicFunction(feasibleNode, endNode);
                Q.push(feasibleNode);
            }

            else if (updatedLength < feasibleNode->lengthFromStart)
            {
                feasibleNode->lengthFromStart = updatedLength;
                feasibleNode->prevNode = currentNode;
                feasibleNode->heuristic = updatedLength + heuristicFunction(feasibleNode, endNode);
                // Q.push(feasibleNode);
            }
        }
    }
}


