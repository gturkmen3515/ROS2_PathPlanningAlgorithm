#ifndef DUMMY_CONTROLLER_HPP
#define DUMMY_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "tf2/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include "carla_msgs/msg/carla_ego_vehicle_control.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_status.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <cmath>

using namespace std::chrono_literals;

class DummyController : public rclcpp::Node
{
    public:
        DummyController();
        virtual ~DummyController();
    
    private:
        // node parameters
        std::string path_topic_name_;
        std::string odom_topic_name_;
        std::string status_topic_name_;

        // PID paramaters
        double referance_speed_;        // reference for pid controller
        double current_speed_;          // current speed measurement
        double Kp_;                     // Proportional constant
        double Ki_;                     // Integral constant
        double Kd_;                     // Derivative constant
        double integral_term_;
        double derivative_term_;
        double last_error_;             // previous error
        double dt_;                     // delta t

        // Pure Pursuit Paramaters
        double wheel_base_;             // distance between the wheels
        double Kdd_;
        double min_ld_;                 // minimum look ahead distance
        double max_ld_;                 // maximum look ahead distance

        std::shared_ptr<nav_msgs::msg::Path> following_path_;
        std::shared_ptr<nav_msgs::msg::Odometry> vehicle_odom_;
        geometry_msgs::msg::PoseStamped pose_stamped_;
        // std::shared_ptr<geometry_msgs::msg::Point> target_point_;

        // target point
        double x_;
        double y_;

        double roll_;
        double pitch_;
        double yaw_;

        int index_;

        bool path_received_=false;
        bool goal_reached_=false;
        // subscriber
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
        rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleStatus>::SharedPtr status_subscriber_;

        // publisher
        rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr control_command_publisher_;

        // timer
        rclcpp::TimerBase::SharedPtr control_command_pub_timer_;

        // functions & callbacks
        // path callback
        void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
        // odom callback
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        // status callback
        void statusCallback(const carla_msgs::msg::CarlaEgoVehicleStatus::SharedPtr msg);
        // ego vehicle lateral control over Pure Pursuit
        double lateralControl(nav_msgs::msg::Path::SharedPtr path, nav_msgs::msg::Odometry::SharedPtr odom);
        // ego vehicle speed controller with PID
        double longitudinalControl();
        // publisher timer
        void timerCallback();
        // map two scale of numbers betrween each other
        double mapValue(double input, double x_min, double x_max, double y_min, double y_max);
        // distance between two point
        double getDistance(geometry_msgs::msg::Pose pose_a, geometry_msgs::msg::Pose pose_b);
        double getDistance(double point_a_x, double point_a_y, double point_b_x, double point_b_y);
};

#endif // DUMMY_CONTROLLER_HPP