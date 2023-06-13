//
// Created by atakan on 08.06.2023.
//

#ifndef CPP_PUBSUB_MAZESOLVER_H
#define CPP_PUBSUB_MAZESOLVER_H

#include "rclcpp/rclcpp.hpp" // ROS C++ API
#include "std_msgs/msg/float64_multi_array.hpp" // ROS message type
#include <iostream> // Input/output stream
#include <vector> // Standard vector container
#include <cmath> // Math functions
#include <algorithm> // Algorithms library
#include "astar.h" // Custom A* algorithm implementation

// Define operator<< for printing vectors
template <typename S>
std::ostream& operator<<(std::ostream& os, const std::vector<S>& vector)
{
    for (auto element : vector) {
        os << element << " ";
    }
    return os;
}

// Define a custom allocator
template <typename T>
class CustomAllocator : public std::allocator<T>
{
public:
    using value_type = T;
    using pointer = T*;
    using const_pointer = const T*;

    pointer allocate(std::size_t n)
    {
        return static_cast<pointer>(::operator new(n * sizeof(T)));
    }

    void deallocate(pointer p, std::size_t n)
    {
        ::operator delete(p, n * sizeof(T));
    }
};

// Define a custom vector type using the custom allocator
template <typename T>
using CustomVector = std::vector<T, CustomAllocator<T>>;

// Define the MazeSolver class, derived from rclcpp::Node
class MazeSolver : public rclcpp::Node
{
public:
    // Constructor
    MazeSolver() : Node("maze_solver_node")
    {
        // Create publishers
        path_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/path_array", 10);

        // Create subscribers
        obsx_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/obsx", 10, std::bind(&MazeSolver::obsxCallback, this, std::placeholders::_1));

        obsy_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/obsy", 10, std::bind(&MazeSolver::obsyCallback, this, std::placeholders::_1));

        gridsize_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/gridsize", 10, std::bind(&MazeSolver::gridsizeCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&MazeSolver::publishData, this));
    }

    // Callback function for obsx subscription
    void obsxCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        obsx_ = *msg;
    }

    // Callback function for obsy subscription
    void obsyCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        obsy_ = *msg;
    }

    // Callback function for gridsize subscription
    void gridsizeCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        gridsize_ = *msg;
    }

    // Function to publish data
    bool isCellOnLine(int x1, int y1, int x2, int y2) {
        int dx = std::abs(x2 - x1);
        int dy = std::abs(y2 - y1);
        int sx = (x1 < x2) ? 1 : -1;
        int sy = (y1 < y2) ? 1 : -1;
        int err = dx - dy;

        while (true) {
            if (x1 == x2 && y1 == y2) {
                return true;  // The cell lies on the line segment
            }

            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x1 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y1 += sy;
            }
        }

        return false;  // The cell does not lie on the line segment
    }

    void publishData()
    {
        // Create and populate the message
        auto width = gridsize_.data[0];
        auto height = gridsize_.data[1];
        auto startx = gridsize_.data[2];
        auto starty = gridsize_.data[3];
        auto endx = gridsize_.data[4];
        auto endy = gridsize_.data[5];

        auto obsy_val = obsy_.data;
        auto obsx_val = obsx_.data;

        auto path_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
        path_msg->layout.dim.push_back(std_msgs::msg::MultiArrayDimension());

        const int ROWS = height;
        const int COLS = width;

        std::vector<std::vector<Node_s*>> grid(ROWS, std::vector<Node_s*>(COLS));

        // Initialize grid and obstacles
        for (int i = 0; i < ROWS; i++) {
            for (int j = 0; j < COLS; j++) {

                grid[i][j] = new Node_s(i, j);
            }
        }

        double vehicleWidth = 3;  // Define the width of the vehicle
        double vehicleLength = 10; // Define the length of the vehicle

        for (int i = 0; i < std::distance(obsx_val.begin(), obsx_val.end()); i++) {
            int obsx = obsx_val[i];
            int obsy = obsy_val[i];

            // Mark the cells along the line segment representing the vehicle's dimensions
            for (int dx = -vehicleWidth; dx <= vehicleWidth; dx++) {
                for (int dy = -vehicleLength; dy <= vehicleLength; dy++) {
                    int x = obsx + dx;
                    int y = obsy + dy;

                    // Check if the cell lies on the line segment
                    if (isCellOnLine(obsx, obsy, x, y)) {
                        if (x >= 0 && x < ROWS && y >= 0 && y < COLS) {
                            grid[x][y]->obstacle = 1;
                        }
                    }
                }
            }
        }

// ...

// Helper function to check if a cell lies on a line segment

        Node_s* startNode = grid[startx][starty];
        Node_s* endNode = grid[endx][endy];

        std::vector<Node_s*> path = AStar(startNode, endNode, grid);
        std::vector<int> x_path;
        std::vector<int> y_path;

        // Print the found path
        if (!path.empty()) {
            std::cout << "Path found:" << std::endl;
            for (Node_s* node : path) {
                std::cout << "(" << node->x << ", " << node->y << ")" << std::endl;
                x_path.push_back(node->x);
                y_path.push_back(node->y);
            }
        } else {
            std::cout << "Path not found." << std::endl;
        }

        // Convert the path to double arrays
        std::vector<double> x_path_d(x_path.begin(), x_path.end());
        std::vector<double> y_path_d(y_path.begin(), y_path.end());
        std::vector<std::pair<std::vector<double>, std::vector<double>>> path_vec;
        path_vec.push_back(std::make_pair(x_path_d, y_path_d));

        std::vector<std::array<double, 2>> path_data;
        path_data.reserve(x_path_d.size());

        for (std::size_t i = 0; i < x_path_d.size(); ++i) {
            path_data.push_back({x_path_d[i], y_path_d[i]});
        }

        for (const auto& point : path_data) {
            path_msg->data.insert(path_msg->data.end(), point.begin(), point.end());
        }

        path_publisher_->publish(*path_msg);
    }



    // Destructor
    ~MazeSolver()
    {
        // Clean up memory
        for (int i = 0; i < ROWS; i++) {
            for (int j = 0; j < COLS; j++) {
                delete grid[i][j];
            }
        }
    }

private:
    // ROS subscribers
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr obsx_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr obsy_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gridsize_subscription_;

    // ROS publisher
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr path_publisher_;

    // ROS timer
    rclcpp::TimerBase::SharedPtr timer_;

    // ROS messages for data exchange
    std_msgs::msg::Float64MultiArray gridsize_;
    std_msgs::msg::Float64MultiArray obsx_;
    std_msgs::msg::Float64MultiArray obsy_;

    // Constants for grid size
    const int ROWS = 0; // Update with actual ROWS value
    const int COLS = 0; // Update with actual COLS value

    // Grid of nodes
    std::vector<std::vector<Node_s*, CustomAllocator<Node_s*>>> grid;

};
#endif //CPP_PUBSUB_MAZESOLVER_H
