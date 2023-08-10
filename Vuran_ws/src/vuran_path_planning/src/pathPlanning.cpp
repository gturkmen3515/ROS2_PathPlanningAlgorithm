//
// Created by atakan on 08.06.2023.
//


#include "rclcpp/rclcpp.hpp" // ROS C++ API
#include "std_msgs/msg/float64_multi_array.hpp" // ROS message type
#include <iostream> // Input/output stream
#include <vector> // Standard vector container
#include <cmath> // Math functions
#include <algorithm> // Algorithms library
#include "vuran_path_planning/astar.h" // Custom A* algorithm implementation
#include <fstream> // File handling
#include "../include/vuran_path_planning/isCellOnLine.h"
#include "../include/vuran_path_planning/smoothing.h"
#include "../include/vuran_path_planning/pathPlanning.h"

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "../include/vuran_path_planning/pathPlanning.h"

// Constructor
PathPlanning::PathPlanning() : Node("maze_solver_node")
{
    // Create subscribers
    obsx_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/obsx", 10, std::bind(&PathPlanning::obsxCallback, this, std::placeholders::_1));

    obsy_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/obsy", 10, std::bind(&PathPlanning::obsyCallback, this, std::placeholders::_1));

    gridsize_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/gridsize", 10, std::bind(&PathPlanning::gridsizeCallback, this, std::placeholders::_1));

    loccation_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/loccation", 10, std::bind(&PathPlanning::loccationCallback, this, std::placeholders::_1));

    // Create publishers
    path_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/path_array", 10);
    smooth_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/waypoints", 10);

    // Create timer
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PathPlanning::publishData, this));
}

// Destructor
PathPlanning::~PathPlanning()
{
    // Clean up memory
    for (int i = 0; i < ROWS; i++) {
        for (int j = 0; j < COLS; j++) {
            delete grid[i][j];
        }
    }
}

// Member function definitions
void PathPlanning::obsxCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    obsx_ = *msg;
}

void PathPlanning::obsyCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    obsy_ = *msg;
}

void PathPlanning::gridsizeCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    gridsize_ = *msg;
    flag = true;
}

void PathPlanning::loccationCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    loccation_ = *msg;
}

void PathPlanning::publishData()
{
    if(flag){
    // Create and populate the message


    //const int car_xloc=loccation_.data[0];
    //const int car_yloc=loccation_.data[1];

    const int ROWS = gridsize_.data[0];
    const int COLS = gridsize_.data[1];
    const int vehicleWidth = 3;  // Define the width of the vehicle
    const int vehicleLength = 3; // Define the length of the vehicle

    std::vector<std::vector<Node_s*>> grid(ROWS, std::vector<Node_s*>(COLS));

    // Initialize grid and obstacles
    for (int i = 0; i < ROWS; i++)
    {
        for (int j = 0; j < COLS; j++)
        {

            grid[i][j] = std::move(new Node_s(i, j,grid,0,0));
        }
    }

    // std::ofstream obsFile("obstacle_conf.txt");
    //if (obsFile.is_open())
    //{
    // for (int i = 0; i < obsx_.data.size(); i++)
    // {
    //     int obsx = std::move(obsx_.data[i]);
    //     int obsy = std::move(obsy_.data[i]);
    //     grid[obsx][obsy]->obstacle = 1;

    // }

        for (int i = 0; i < obsx_.data.size(); i++)
        {
            int obsx = std::move(obsx_.data[i]);
            int obsy = std::move(obsy_.data[i]);

            // Mark the cells along the line segment representing the vehicle's dimensions
            for (int dx = -vehicleLength/2; dx <= vehicleLength/2; dx++) {
                for (int dy = -vehicleWidth/2; dy <= vehicleWidth/2; dy++) {
                    int x = obsx + dx;
                    int y = obsy + dy;

                    // Check if the cell lies on the line segment
                    if (isCellOnLine(obsx, obsy, x, y)) {
                        if (x >= 0 && x < ROWS && y >= 0 && y < COLS) {
                            grid[x][y]->obstacle = 1;
                            //obsFile << x  << "," << y << "\n";

                        }
                    }
                }
            }
        }
    //}
    //obsFile.close();
    //std::cout << "Obstacle written to obstacle.txt" << std::endl;
    Node_s* startNode = grid[gridsize_.data[2]][gridsize_.data[3]];
    Node_s* endNode = grid[gridsize_.data[4]][gridsize_.data[5]];
    int car_index=gridsize_.data[8];
    int goal_index=gridsize_.data[9];

    std::vector<Node_s*> path = AStar(startNode, endNode, grid,car_index,goal_index);
    // Clear path_msg->data before populating
    path_msg->data.clear();


    nav_msgs::msg::Path smothed_path_msg;
    smothed_path_msg.header.frame_id="map";
    if (!path.empty())
    {
        std::cout << "Path found:" << std::endl;

        std::vector<std::vector<double>> path_array;
        path_array.reserve(path.size());
        int counter = 0;

        for (Node_s* node : path)
        {
            if (counter % 2 == 0)
            {
            //std::cout << "(" << node->x << ", " << node->y << ")" << std::endl;
            path_array.push_back(std::move(std::vector<double>{node->x, node->y}));
           }

            counter++;
        }

        // Set the dimensions of the path message
        path_msg->layout.dim.clear();
        path_msg->layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        path_msg->layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        path_msg->layout.dim[0].label = "rows";
        path_msg->layout.dim[0].size = path_array.size();
        path_msg->layout.dim[0].stride = path_array.size() * 2;
        path_msg->layout.dim[1].label = "cols";
        path_msg->layout.dim[1].size = 2;
        path_msg->layout.dim[1].stride = 2;

        int resolution = 50;  // Number of interpolated points between each pair of original points

        BSpline bSpline(path_array);
        std::vector<std::vector<double>> smoothedPath = bSpline.getSmoothPath(resolution);

        // Flatten the 2D vector into a 1D array
        //for (const auto& point : path_array)
        
        geometry_msgs::msg::PoseStamped pose_stamped_msg;
        pose_stamped_msg.header.frame_id="map";
        pose_stamped_msg.header.stamp = rclcpp::Clock{}.now();
        //std::cout << smoothedPath.size()<<std::endl;
        //for (const auto& point : path_array)
        for (const auto& point : smoothedPath)
        {
            path_msg->data.insert(path_msg->data.end(), point.begin(), point.end());
            //geometry_msgs::msg::PoseStamped pose;
            //pose.pose.position.x=point[0];
            //pose.pose.position.y=point[1];
            // Fill the pose data (you need to set your desired values here)



            pose_stamped_msg.pose.position.x = point[0]+gridsize_.data[6];
            pose_stamped_msg.pose.position.y = point[1]+gridsize_.data[7];
            pose_stamped_msg.pose.position.z = 0.0;
            //std::cout<<"X pos:"<<point[0]+gridsize_.data[6]<<"Y pos:" << point[1]+gridsize_.data[7]<< std::endl;

            smothed_path_msg.poses.push_back(pose_stamped_msg);

        }
            //std::cout<<"-----------------------------------------" << std::endl;

        // std::ofstream outFile("path_smooth.txt");
        // if (outFile.is_open())
        // {
        //     for (size_t i = 0; i < path_msg->data.size(); i += 2) {
        //         outFile << path_msg->data[i] << "," << path_msg->data[i + 1] << "\n";
        //     }
        //     outFile.close();
        //     std::cout << "Path written to path.txt" << std::endl;
        // }
        // else
        // {
        //     std::cout << "Failed to open path.txt for writing" << std::endl;
        // }

    }
    else
    {
        std::cout << "Path not found." << std::endl;
    }


    //std::cout << "smothed_path_msg.size     " << smothed_path_msg.poses.size() << std::endl;
    smooth_path_publisher_->publish(smothed_path_msg);
}
}








