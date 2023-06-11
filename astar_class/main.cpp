#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include "matplotlibcpp.h"
#include <iterator>

//#include <queue>
namespace plt=matplotlibcpp;

template <typename S>
std::ostream& operator<<(std::ostream& os,const std::vector<S>& vector)
{
    // Printing all the elements
    // using <<
    for (auto element : vector) {
        os << element << " ";
    }
    return os;
}

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

int main() {
    // 2D nodes grids oluştur
    const int ROWS = 10;
    const int COLS = 10;
    std::vector<std::vector<Node*>> grid(ROWS, std::vector<Node*>(COLS));

    // nodes grid'i ve engelleri başlat
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            grid[i][j] = new Node(i, j);
        }
    }
    // Engelleri ayarla
    std::vector<int> ox;
    std::vector<int> oy;

    for (int i = 1; i <=9; i++) {
        oy.push_back(i);
        ox.push_back(2);
    }

    for (int i = 0; i < 8; i++) {
        oy.push_back(i);
        ox.push_back(4);
    }

    for (int i = 1; i <= 9; i++) {
        oy.push_back(i);
        ox.push_back(6);
    }
    for (int i = 0; i <= 8; i++) {
        oy.push_back(i);
        ox.push_back(8);
    }

   //for(int i=0;i<=sizeof(ox);i++)
    std::vector<double> oxx;
    std::vector<double> oyy;
    for(int i=0;i<std::distance(ox.begin(),ox.end());i++)
        {
        grid[(ox[i])][(oy[i])]->obstacle = 1;
        oxx.push_back((ox[i]-0.01));
        oyy.push_back((oy[i]));
        oxx.push_back((ox[i]+0.01));
        oyy.push_back((oy[i]));
    }


    //std::transform(ox.begin(), ox.end(), oxx.begin(), [](int x) { return (double)x;});
    // Başlangıç ve bitiş noktalarını belirle
    Node* startNode = grid[0][9];
    Node* endNode = grid[9][0];

    // A* algoritmasını kullanarak yol bul
    std::vector<Node*> path = AStar(startNode, endNode, grid);
    std::vector<int> x_path;
    std::vector<int> y_path;

    std::vector<int> start_px;
    std::vector<int> start_py;


    // Bulunan yolu yazdır
    if (!path.empty()) {
        std::cout << "Path found:" << std::endl;

        //std::cout<<"Path size:"<<size(path)<<"\n";
        for (Node* node : path) {
            std::cout << "(" << node->x << ", " << node->y << ")" << std::endl;
            x_path.push_back(node->x);
            y_path.push_back(node->y);
        }
    }

    else {
        std::cout << "Path not found."  << std::endl;
    }

    start_px.push_back(*(x_path.begin()));
    start_px.push_back(*(x_path.end()-1));

    start_py.push_back(*(y_path.begin()));
    start_py.push_back(*(y_path.end()-1));

    plt::plot(x_path,y_path);
    plt::scatter(start_px,start_py,{{"color","orange"}, {"marker","*"} ,{"linewidth","20"}});
    plt::scatter(ox,oy,{{"color","red"}, {"marker","_"},{"linewidth","50"}});
    plt::scatter(oxx,oyy,{{"color","red"}, {"marker","_"},{"linewidth","50"}});

    plt::grid(true);
    plt::savefig("result.png");
    plt::show();

    // Belleği temizle
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            delete grid[i][j];
        }
    }


    return 0;
}
/*
 grid[2][1]->obstacle = 1;
 grid[2][2]->obstacle = 1;
 grid[2][3]->obstacle = 1;
 grid[2][4]->obstacle = 1;
 grid[2][5]->obstacle = 1;
 grid[2][6]->obstacle = 1;
 grid[2][7]->obstacle = 1;
 grid[2][8]->obstacle = 1;
 grid[2][9]->obstacle = 1;

 grid[4][0]->obstacle = 1;
 grid[4][1]->obstacle = 1;
 grid[4][2]->obstacle = 1;
 grid[4][3]->obstacle = 1;
 grid[4][4]->obstacle = 1;
 grid[4][5]->obstacle = 1;
 grid[4][6]->obstacle = 1;
 grid[4][7]->obstacle = 1;
 grid[4][8]->obstacle = 1;

 grid[6][1]->obstacle = 1;
 grid[6][2]->obstacle = 1;
 grid[6][3]->obstacle = 1;
 grid[6][4]->obstacle = 1;
 grid[6][5]->obstacle = 1;
 grid[6][6]->obstacle = 1;
 grid[6][7]->obstacle = 1;
 grid[6][8]->obstacle = 1;
 grid[6][9]->obstacle = 1;

 grid[8][0]->obstacle = 1;
 grid[8][1]->obstacle = 1;
 grid[8][2]->obstacle = 1;
 grid[8][3]->obstacle = 1;
 grid[8][4]->obstacle = 1;
 grid[8][5]->obstacle = 1;
 grid[8][6]->obstacle = 1;
 grid[8][7]->obstacle = 1;
 grid[8][8]->obstacle = 1;
*/
/*
    //std::vector<std::vector<double>> grid_path={};
    //grid_path.push_back(grid);
//std::cout<<x_path;
for (int i=0;i<ROWS;i++){
    for(int j=0;j<COLS;j++){
        if (grid[i][j]->obstacle){
            ox.push_back(i);
            oy.push_back(j);
        }

    }
}
*/