//
// Created by atakan on 08.06.2023.
//
//
// Created by atakan on 08.06.2023.
//

// Include necessary libraries and headers
#include "rclcpp/rclcpp.hpp" // ROS C++ API
#include "std_msgs/msg/float64_multi_array.hpp" // ROS message type
#include <iostream> // Input/output stream
#include <vector> // Standard vector container
#include <cmath> // Math functions
#include <algorithm> // Algorithms library
#include "MazeSolver.h"


// Main function
int main(int argc, char** argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);

    // Create an instance of the MazeSolver class
    auto node = std::make_shared<MazeSolver>();

    // Spin the node, i.e., start processing callbacks
    rclcpp::spin(node);

    // Shutdown ROS
    rclcpp::shutdown();

    return 0;
}
