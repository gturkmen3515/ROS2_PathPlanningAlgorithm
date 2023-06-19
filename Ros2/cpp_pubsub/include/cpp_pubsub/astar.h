//
// Created by atakan on 29.05.2023.
//

#ifndef UNTITLED_ASTAR_H
#define UNTITLED_ASTAR_H
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iterator>

//#include <queue>

class Node_s {
public:
    int x, y; // position of the node
    double g, h, f; // g score, h score, and f score
    bool obstacle; // 1 if the node is an obstacle
    Node_s* parent; // parent node in the path

    Node_s(int x, int y) : x(x), y(y), g(0), h(0), f(0), obstacle(false), parent(nullptr) {
    }
    ~Node_s(){
        delete parent;
    }
};
// iki nodes arasındaki euler mesafe
double distance(Node_s* a, Node_s* b) ;

// A* pathfinding algorithm;
std::vector<Node_s*> AStar(Node_s* startNode, Node_s* endNode, std::vector<std::vector<Node_s*>>& grid) ;

#endif //UNTITLED_ASTAR_H
