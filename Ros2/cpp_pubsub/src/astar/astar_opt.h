//
// Created by atakan on 03.07.2023.
//
#ifndef UNTITLED9_ASTAR_NEW_H
#define UNTITLED9_ASTAR_NEW_H

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <unordered_set>
class Node_s {
public:
    int x, y;
    double g, h, f;
    bool obstacle;
    Node_s* parent;
    std::vector<std::vector<Node_s*>>& grid;

    Node_s(int x, int y, std::vector<std::vector<Node_s*>>& grid)
            : x(x), y(y), g(0), h(0), f(0), obstacle(false), parent(nullptr), grid(grid) {
    }

    ~Node_s() {
        delete parent;
    }

    double squaredDistance(Node_s* other) {
        return std::pow(x - other->x, 2) + std::pow(y - other->y, 2);
    }

    Node_s* getNeighbor(int dx, int dy) {
        int newX = x + dx;
        int newY = y + dy;
        if (newX >= 0 && newX < grid.size() && newY >= 0 && newY < grid[newX].size()) {
            return grid[newX][newY];
        }
        return nullptr;
    }
};

class CompareNodes {
public:
    bool operator()(const Node_s* a, const Node_s* b) const {
        return a->f > b->f;
    }
};

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
            if (neighbor == nullptr || neighbor->obstacle || closedSet.count(neighbor) > 0) {
                continue;
            }

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

    return path;
}

#endif //UNTITLED9_ASTAR_NEW_H
