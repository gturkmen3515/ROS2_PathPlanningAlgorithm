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

class Node {
public:
    int x, y; // position of the node
    double g, h, f; // g score, h score, and f score
    bool obstacle; // 1 if the node is an obstacle
    Node* parent; // parent node in the path

    Node(int x, int y) : x(x), y(y), g(0), h(0), f(0), obstacle(false), parent(nullptr) {
    }
    ~Node(){
        delete parent;
        std::cout<<"all message cleaned"<<"\n";

    }
};

// iki nodes arasındaki euler mesafe
double distance(Node* a, Node* b) {
    return (std::sqrt(std::pow(a->x - b->x, 2) + std::pow(a->y - b->y, 2)));
}

// A* pathfinding algorithm
std::vector<Node*> AStar(Node* startNode, Node* endNode, std::vector<std::vector<Node*>>& grid) {
    std::vector<Node*> path; // başlangıçtan sona doğru çıkarılan path

    std::vector<Node*> openList; // gidilmemiş nodes
    std::vector<Node*> closedList; // gidilmiş nodes

    openList.push_back(startNode);

    while (!openList.empty()) {
        // en düşük f puanına sahip nodu al
        Node* currentNode = openList[0];
        for (int i = 1; i < openList.size(); i++) {
            if (openList[i]->f < currentNode->f) {//şuanki bulunduğum nodun f scoru openList'in f scorundan büyükse şuanki node openlist node'una eşit oldu
                currentNode = openList[i];
            }
            if (openList[i]->f > currentNode->f) {//şuanki bulunduğum nodun f scoru openList'in f scorundan küçükse bir sonraki for loop iterasyonuna geç
                continue;
            }
        }

        //şuanki nodu openList'den kaldırın ve closedList'e ekleyin
        openList.erase(std::remove(openList.begin(), openList.end(), currentNode), openList.end());
        closedList.push_back(currentNode);//şuanki node gidilmiş node olarak işlendi

        // şuanki node son node'e eşitlendi, path bulundu pathi için geri git elde edileni pathi ters çevir
        if (currentNode == endNode) {
            Node* current = currentNode;
            while (current != startNode) {
                path.push_back(current);
                current = current->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        // yapılacak 8 hareket için ulaşılabilineçek children
        std::vector<Node*> children;
        if (currentNode->x > 0 && !grid[currentNode->x - 1][currentNode->y]->obstacle) {
            children.push_back(grid[currentNode->x - 1][currentNode->y]);
        }
        if (currentNode->x > 0 && currentNode->y > 0 && !grid[currentNode->x-1][currentNode->y - 1]->obstacle) {
            children.push_back(grid[currentNode->x-1][currentNode->y - 1]);
        }

        if (currentNode->x > 0 && currentNode->y < grid[0].size() - 1 && !grid[currentNode->x-1][currentNode->y + 1]->obstacle) {
            children.push_back(grid[currentNode->x-1][currentNode->y + 1]);
        }
        if (currentNode->x < grid.size() - 1 && !grid[currentNode->x + 1][currentNode->y]->obstacle) {
            children.push_back(grid[currentNode->x + 1][currentNode->y]);
        }
        if (currentNode->x < grid.size() - 1 && currentNode->y < grid[0].size() - 1 && !grid[currentNode->x+1][currentNode->y + 1]->obstacle) {
            children.push_back(grid[currentNode->x+1][currentNode->y + 1]);
        }


        if (currentNode->x < grid.size() - 1 && currentNode->y > 0 && !grid[currentNode->x+1][currentNode->y - 1]->obstacle) {
            children.push_back(grid[currentNode->x+1][currentNode->y - 1]);
        }

        if (currentNode->y > 0 && !grid[currentNode->x][currentNode->y - 1]->obstacle) {
            children.push_back(grid[currentNode->x][currentNode->y - 1]);
        }
        if (currentNode->y < grid[0].size() - 1 && !grid[currentNode->x][currentNode->y + 1]->obstacle) {
            children.push_back(grid[currentNode->x][currentNode->y + 1]);
        }/*
        else if (grid[currentNode->x][currentNode->y ]->obstacle) {
            children.remove((grid[currentNode->x][currentNode->y]));
        }*/


        for (Node* child : children) {
            //child önceden uğranmış node da ise for loopu 1 iter ilerlet o nodeları hesaba katma
            if (std::find(closedList.begin(), closedList.end(), child) != closedList.end()) {
                continue;
            }

            // g, h, and f score'ları hesapla
            double gScore = currentNode->g + distance(currentNode, child);
            double hScore = distance(child, endNode);
            double fScore = gScore + hScore;
            auto it = std::find(openList.begin(), openList.end(), child);//openList'in içinden child'ın bulunduğu iterasyonu bul
            if (it != openList.end()) {//child sona ulaşıldı mı diye kontrol et
                // şuanki düğüme ulaşıldığında child'ın g puanı daha düşükse, it'in ebeveynini, g  ve f puanını güncelle
                if (fScore < (*it)->f) {
                    (*it)->parent = currentNode;
                    (*it)->g = gScore;
                    (*it)->h = hScore;
                    (*it)->f = fScore;
                }
            }
            else { // child'ı open liste ekle
                child->g = gScore;
                child->h = hScore;
                child->f = fScore;
                child->parent = currentNode;
                openList.push_back(child);
            }
        }
    }

    // son düğüme giden bir yol yoksa, boş bir vektör döndürür
    return path;
}
#endif //UNTITLED_ASTAR_H
