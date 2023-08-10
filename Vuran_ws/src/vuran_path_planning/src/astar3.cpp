#include "../include/vuran_path_planning/astar.h"

std::vector<Node_s*> AStar(Node_s* startNode, Node_s* endNode, std::vector<std::vector<Node_s*>>& grid,int car_index,int goal_index) {
    std::vector<Node_s*> path;
    std::vector<int> possibleMoves;

    std::priority_queue<Node_s*, std::vector<Node_s*>, CompareNodes> openList;
    std::unordered_set<Node_s*> closedSet;
    int dxValues[] = {1,1,0,-1,-1,-1, 0, 1};
    int dyValues[] = {0,1,1, 1, 0,-1,-1,-1};

    startNode->f = 0;
    openList.push(startNode);
    int prevMotionIndex = car_index;  // Initialize with an invalid index
    //int step_size=abs(goal_index-car_index)+1;
    int step_size=2;

    int index_count=0;
    Node_s* prevClosedNode = nullptr;

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

        if (prevClosedNode != nullptr && index_count>30) {
            step_size=1;
            int dx = currentNode->x - prevClosedNode->x;
            int dy = currentNode->y - prevClosedNode->y;

            //prevMotionIndex = -1;
            for (int i = 0; i < 8; i++) {
                if (dx == dxValues[i] && dy == dyValues[i]) {
                    prevMotionIndex = i;
                    break;
                }
            }
        }
        //std::cout<<"prevMotionIndex: "<<prevMotionIndex<<"\n";
        // Define possible moves based on the previous motion index

     
        possibleMoves = {(prevMotionIndex + 6) % 8,(prevMotionIndex + 7) % 8, prevMotionIndex, (prevMotionIndex + 1) % 8, (prevMotionIndex + 2) % 8};
        

        for (int moveIndex : possibleMoves) {
            int dx = dxValues[moveIndex]*step_size;
            int dy = dyValues[moveIndex]*step_size;

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
        index_count++;
        prevClosedNode = currentNode;
    }

    return path;
}
