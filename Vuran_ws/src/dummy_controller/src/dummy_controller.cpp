#include "dummy_controller/dummy_controller.hpp"

DummyController::DummyController() : Node("dummy_controller")
{
    integral_term_ = 0.0;
    derivative_term_ = 0.0;
    last_error_ = 0.0;
    dt_ = 0.01;

    x_ = 0.0;
    y_ = 0.0;

    index_ = 0;

    // decleare node params
    declare_parameter<std::string>("topics.path_topic_name", "path_topic_name_");
    declare_parameter<std::string>("topics.odom_topic_name", "odom_topic_name_");
    declare_parameter<std::string>("topics.status_topic_name", "status_topic_name_");
    declare_parameter<double>("ego_vehicle.target_speed", 0.5);
    declare_parameter<double>("PID.Kp", 1.0);
    declare_parameter<double>("PID.Ki", 1.0);
    declare_parameter<double>("PID.Kd", 1.0);
    // Pure Pursuit params
    declare_parameter<double>("PurePursuit.wheel_base", 2.0);
    declare_parameter<double>("PurePursuit.Kdd", 1.0);
    declare_parameter<double>("PurePursuit.min_ld", 5.0);
    declare_parameter<double>("PurePursuit.max_ld", 5.0);

    // get node params
    path_topic_name_ = get_parameter("topics.path_topic_name").as_string();
    odom_topic_name_ = get_parameter("topics.odom_topic_name").as_string();
    status_topic_name_ = get_parameter("topics.status_topic_name").as_string();
    referance_speed_ = get_parameter("ego_vehicle.target_speed").as_double();
    Kp_ = get_parameter("PID.Kp").as_double();
    Ki_ = get_parameter("PID.Ki").as_double();
    Kd_ = get_parameter("PID.Kd").as_double();
    // Pure Pursuit params
    wheel_base_ = get_parameter("PurePursuit.wheel_base").as_double();
    Kdd_ = get_parameter("PurePursuit.Kdd").as_double();
    min_ld_ = get_parameter("PurePursuit.min_ld").as_double();
    max_ld_ = get_parameter("PurePursuit.max_ld").as_double();

    // RCLCPP_WARN(this->get_logger(), "Path topic name: %s", path_topic_name_.c_str());
    // RCLCPP_WARN(this->get_logger(), "Odom topic name: %s", odom_topic_name_.c_str());
    // RCLCPP_WARN(this->get_logger(), "Status topic name: %s", status_topic_name_.c_str());
    // RCLCPP_WARN(this->get_logger(), "Target speed: %f", referance_speed_);
    // RCLCPP_WARN(this->get_logger(), "Kp: %f", Kp_);
    // RCLCPP_WARN(this->get_logger(), "Ki: %f", Ki_);
    // RCLCPP_WARN(this->get_logger(), "Kd: %f", Kd_);
    // RCLCPP_WARN(this->get_logger(), "Wheel base: %f", wheel_base_);
    // RCLCPP_WARN(this->get_logger(), "Kdd: %f", Kdd_);
    // RCLCPP_WARN(this->get_logger(), "min_ld: %f", min_ld_);
    // RCLCPP_WARN(this->get_logger(), "max_ld : %f", max_ld_);

    path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>("waypoints", 1, std::bind(&DummyController::pathCallback, this, std::placeholders::_1));
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("carla/ego_vehicle/odometry",3, std::bind(&DummyController::odomCallback, this, std::placeholders::_1));
    status_subscriber_ = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleStatus>("carla/ego_vehicle/vehicle_status",3, std::bind(&DummyController::statusCallback, this, std::placeholders::_1));
    control_command_publisher_ = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>("carla/ego_vehicle/vehicle_control_cmd", 10);
    control_command_pub_timer_ = this->create_wall_timer(500ms, std::bind(&DummyController::timerCallback, this));
}

DummyController::~DummyController()
{
}

void DummyController::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Path Callback !!!");
    following_path_ = std::move(msg);
    path_received_ = true;
    RCLCPP_ERROR(this->get_logger(), "PATH RECEIVED !!!");

}

void DummyController::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    //RCLCPP_INFO(this->get_logger(), "Odom Callback !!!");
    vehicle_odom_ = std::move(msg);

    tf2::Quaternion quaternion=  tf2::Quaternion(vehicle_odom_->pose.pose.orientation.x,
                                                    vehicle_odom_->pose.pose.orientation.y,
                                                    vehicle_odom_->pose.pose.orientation.z,
                                                    vehicle_odom_->pose.pose.orientation.w);
    tf2::getEulerYPR(quaternion,yaw_, pitch_, roll_);
    // tf2::fromMsg(*msg,quaternion);
    //tf2::Matrix3x3(quaternion).getRPY(roll_, pitch_, yaw_);

    // current_speed_ = vehicle_odom_->twist.twist.linear.x;
    // RCLCPP_WARN(this->get_logger(), "Ego Vehivle Speed ==> %f", current_speed_);
}

void DummyController::statusCallback(const carla_msgs::msg::CarlaEgoVehicleStatus::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Status Callback !!!");
    current_speed_ = mapValue(msg->velocity, 0, 12, 0.0, 0.5);
    // RCLCPP_WARN(this->get_logger(), "Ego Vehivle Speed ==> %f", current_speed_);
}

double DummyController::lateralControl(nav_msgs::msg::Path::SharedPtr path, nav_msgs::msg::Odometry::SharedPtr odom)
{
    index_ = (path->poses.back().pose.position.x == pose_stamped_.pose.position.x && path->poses.back().pose.position.y == pose_stamped_.pose.position.y) ? index_ : 0;

    for(index_; index_ < path->poses.size(); index_++)
    {
        auto target_distance = getDistance(path->poses[index_].pose,odom->pose.pose);

        if(target_distance >= min_ld_)
        {
            x_ = path->poses[index_].pose.position.x;
            y_ = path->poses[index_].pose.position.y;
            break;
        }
    }
    goal_reached_=((path->poses.size()-index_)<3)? true:false;
    path_received_=(goal_reached_) ? false:true;
    auto ld = getDistance(odom->pose.pose.position.x,odom->pose.pose.position.y,x_,y_ );
    
    // RCLCPP_INFO(this->get_logger(), "Target Point ====> X ====> %f Y ====> %f",x_, y_);

    auto delta_x = x_ - odom->pose.pose.position.x;
    auto delta_y = y_ - odom->pose.pose.position.y;

    auto alfa = std::atan2(delta_y, delta_x) - yaw_;

    RCLCPP_INFO(this->get_logger(), "Alfa ====> %f", alfa);

    RCLCPP_ERROR(this->get_logger(), "STEER_ANGLE ====> %f", std::atan2(2 * wheel_base_ * std::sin(alfa) , ld));
    pose_stamped_.pose.position.x=path->poses.back().pose.position.x;
    pose_stamped_.pose.position.y=path->poses.back().pose.position.y;

    return -std::atan2(2 * wheel_base_ * std::sin(alfa) , ld);
    
}

double DummyController::longitudinalControl()
{
    // calculate error
    auto error = referance_speed_ - current_speed_;
    
    // calculate integral term
    integral_term_ += error * Ki_ * dt_;

    if(last_error_ != 0)
    {
        derivative_term_ = (error - last_error_)/dt_ * Kd_;
    }

    last_error_ = error;
    
    return Kp_ * error*0.5 + 0*integral_term_ + 0*derivative_term_;
}

void DummyController::timerCallback()
{
        std::cout<<"index_:"<<index_<<"\n";

    auto ego_vehicle_commands = carla_msgs::msg::CarlaEgoVehicleControl();
    ego_vehicle_commands.header.frame_id = "ego_vehicle";
    ego_vehicle_commands.header.stamp = this->get_clock()->now();
    ego_vehicle_commands.throttle = ((path_received_ &&!goal_reached_))? longitudinalControl():0.0;
    ego_vehicle_commands.steer = (path_received_) ? lateralControl(following_path_, vehicle_odom_) : 0.0;
    ego_vehicle_commands.brake = (!goal_reached_)? 0.0:1;
    ego_vehicle_commands.hand_brake =  false;
    ego_vehicle_commands.reverse = false;
    //std::cout<<"path_received_:"<<path_received_<<"\n";
    //RCLCPP_ERROR(this->get_logger(), "Throttle ==> %f   Steer ==> %f", ego_vehicle_commands.throttle, ego_vehicle_commands.steer);
    control_command_publisher_->publish(ego_vehicle_commands);
}

double DummyController::mapValue(double input, double x_min, double x_max, double y_min, double y_max)
{
    return (input - x_min) * (y_max - y_min) / (x_max - x_min) + y_min;
}

double DummyController::getDistance(geometry_msgs::msg::Pose pose_a, geometry_msgs::msg::Pose pose_b)
{
    return sqrt(pow(pose_a.position.x - pose_b.position.x, 2) + pow(pose_a.position.y - pose_b.position.y, 2) + pow(pose_a.position.z - pose_b.position.z, 2));
}

double DummyController::getDistance(double point_a_x, double point_a_y, double point_b_x, double point_b_y)
{
    return sqrt(pow(point_a_x - point_b_x, 2) + pow(point_a_y - point_b_y, 2));
}
