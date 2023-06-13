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
#include "MazeSolver3.h"

// Global variable to track if the program is interrupted
bool interrupted = false;

// Signal handler function for Ctrl+C
void signalHandler(int signum)
{
    // Set the interrupted flag to true
    interrupted = true;
}

// Main function
int main(int argc, char** argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);

    // Create an instance of the MazeSolver class
    auto node = std::make_shared<MazeSolver>();

    // Set up the signal handler for Ctrl+C
    signal(SIGINT, signalHandler);

    // Spin the node until interrupted
    while (!interrupted && rclcpp::ok())
    {
        rclcpp::spin_some(node);
    }

    // Shutdown ROS
    rclcpp::shutdown();

    return 0;
}
