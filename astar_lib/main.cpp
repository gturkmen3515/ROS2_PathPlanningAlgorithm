#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include "matplotlibcpp.h"
#include <iterator>
#include "astar.h"
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
    Node* startNode = grid[9][0];
    Node* endNode = grid[0][9];

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
