//
// Created by deniz on 05.07.2023.
//
#include "rclcpp/rclcpp.hpp" // ROS C++ API
#include "std_msgs/msg/float64_multi_array.hpp" // ROS message type
#include <iostream> // Input/output stream
#include <vector> // Standard vector container
#include <cmath> // Math functions
#include <algorithm> // Algorithms library
#include "../include/vuran_path_planning/astar.h"
#include "../include/vuran_path_planning/pathPlanning.h"

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

    // Create an instance of the PathPlanning class
    auto node = std::make_shared<PathPlanning>();

    // Set up the signal handler for Ctrl+C
    signal(SIGINT, signalHandler);

    // Spin the node until interrupted
    while (!interrupted && rclcpp::ok())
    {
        rclcpp::spin(node);
    }

    // Shutdown ROS
    rclcpp::shutdown();

    return 0;
}
