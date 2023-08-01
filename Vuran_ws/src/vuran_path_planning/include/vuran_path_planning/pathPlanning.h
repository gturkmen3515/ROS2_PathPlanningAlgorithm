//
// Created by atakan on 08.06.2023.
//

#ifndef CPP_PUBSUB_PATHPLANNING_H
#define CPP_PUBSUB_PATHPLANNING_H

#include "rclcpp/rclcpp.hpp" // ROS C++ API
#include "std_msgs/msg/float64_multi_array.hpp" // ROS message type
#include <iostream> // Input/output stream
#include <vector> // Standard vector container
#include <cmath> // Math functions
#include <algorithm> // Algorithms library
#include <fstream> // File handling
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

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

// Define the PathPlanning class, derived from rclcpp::Node
class PathPlanning : public rclcpp::Node
{
public:
    // Constructor
    PathPlanning();
    ~PathPlanning();

private:
    // ROS subscribers
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr obsx_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr obsy_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gridsize_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr loccation_subscription_;

    // ROS publishers
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr path_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smooth_path_publisher_;

    // ROS timer
    rclcpp::TimerBase::SharedPtr timer_;

    // ROS messages for data exchange
    std_msgs::msg::Float64MultiArray gridsize_;
    std_msgs::msg::Float64MultiArray obsx_;
    std_msgs::msg::Float64MultiArray obsy_;
    std_msgs::msg::Float64MultiArray loccation_;

    // Constants for grid size
    int ROWS = 0; // Update with actual ROWS value
    int COLS = 0; // Update with actual COLS value

    bool flag = false;

    // Grid of nodes
    std::vector<std::vector<Node_s*, CustomAllocator<Node_s*>>> grid;

    // Path message
    std::shared_ptr<std_msgs::msg::Float64MultiArray> path_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();

    // Smoothened path message
    std::shared_ptr<nav_msgs::msg::Path> smooth_path_msg = std::make_shared<nav_msgs::msg::Path>();

    // Member function declarations
    void obsxCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void obsyCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void gridsizeCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void loccationCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void publishData();
};

#endif // CPP_PUBSUB_PATHPLANNING_H

