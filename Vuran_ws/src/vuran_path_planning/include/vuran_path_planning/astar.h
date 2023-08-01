//
// Created by deniz on 05.07.2023.
//

#ifndef SRC_ASTAR_H
#define SRC_ASTAR_H
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

std::vector<Node_s*> AStar(Node_s* startNode, Node_s* endNode, std::vector<std::vector<Node_s*>>& grid);

#endif //SRC_ASTAR_H
