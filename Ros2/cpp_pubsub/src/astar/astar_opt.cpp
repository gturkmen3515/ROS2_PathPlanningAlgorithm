//
// Created by deniz on 05.07.2023.
//

#include "astar_opt.h"

std::vector<Node_s*> AStar(Node_s* startNode, Node_s* endNode, std::vector<std::vector<Node_s*>>& grid) {
    std::vector<Node_s*> path;

    std::priority_queue<Node_s*, std::vector<Node_s*>, CompareNodes> openList;
    std::unordered_set<Node_s*> closedSet;

    startNode->f = 0;
    openList.push(startNode);

    while (!openList.empty()) {
        Node_s* currentNode = openList.top();
        openList.pop();
        closedSet.insert(currentNode);

        if (currentNode == endNode) {
            Node_s* current = currentNode;
            while (current != startNode) {
                path.push_back(current);
                current = current->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        int dxValues[] = {-1, 0, 1, -1, 1, -1, 0, 1};
        int dyValues[] = {-1, -1, -1, 0, 0, 1, 1, 1};

        for (int i = 0; i < 8; i++) {
            int dx = dxValues[i];
            int dy = dyValues[i];

            Node_s* neighbor = currentNode->getNeighbor(dx, dy);

            if (!(neighbor == nullptr || neighbor->obstacle || closedSet.count(neighbor) > 0)) {
                double gScore = currentNode->g + currentNode->squaredDistance(neighbor);
                double hScore = neighbor->squaredDistance(endNode);
                double fScore = gScore + hScore;

                if (neighbor->parent == nullptr || fScore < neighbor->f) {
                    neighbor->parent = currentNode;
                    neighbor->g = gScore;
                    neighbor->h = hScore;
                    neighbor->f = fScore;

                    openList.push(neighbor);
                }
            }
        }
    }

    return path;
}
