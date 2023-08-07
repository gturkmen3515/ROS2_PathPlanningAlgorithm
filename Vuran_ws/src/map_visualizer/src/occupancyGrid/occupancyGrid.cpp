#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "../include/osmVisualizer/dilation.h"

class OccupancyGridPublisher : public rclcpp::Node {
public:
    OccupancyGridPublisher() : Node("occupancy_grid_publisher") {
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("carla/ego_vehicle/odometry",3, std::bind(&OccupancyGridPublisher::odomCallback, this, std::placeholders::_1));
        osm_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/array", 10, std::bind(&OccupancyGridPublisher::float64MultiArrayCallback, this, std::placeholders::_1));
        occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid",10);
        obsx_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("obsx",10);
        obsy_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("obsy",10);
        grid_size_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("gridsize",10);
        goal_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&OccupancyGridPublisher::goalCallback, this, std::placeholders::_1));
        initial_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10, std::bind(&OccupancyGridPublisher::initialCallback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&OccupancyGridPublisher::publishOccupancyGrid, this));
    }

private:
    void publishOccupancyGrid() 
    {
        if((first_ && !array_data_right_y.empty()  && flag2) || prev_count_ != count_)
        {
            int min_x = std::min(*min_element(array_data_left_x.begin(), array_data_left_x.end()),
                                *min_element(array_data_right_x.begin(), array_data_right_x.end()));
            int max_x = std::max(*max_element(array_data_left_x.begin(), array_data_left_x.end()),
                                *max_element(array_data_right_x.begin(), array_data_right_x.end()));
            int min_y = std::min(*min_element(array_data_left_y.begin(), array_data_left_y.end()),
                                *min_element(array_data_right_y.begin(), array_data_right_y.end()));
            int max_y = std::max(*max_element(array_data_left_y.begin(), array_data_left_y.end()),
                                *max_element(array_data_right_y.begin(), array_data_right_y.end()));

            int width = max_x - min_x; // max_x - min_x
            int height = max_y - min_y; // max_y - min_y

            std::vector<int32_t> int_vector_x;
            std::vector<int32_t> int_vector_y;
            for (size_t i = 0; i < array_data_left_x.size(); ++i) 
            {
                // Calculate dist, interval, and num_points
                double dist = getDistance(array_data_left_x[i], array_data_right_x[i], array_data_left_y[i], array_data_right_y[i]);
                double interval = 0.1;
                int num_points = round(dist / interval);
                int x2 = array_data_right_x[i];
                int x1 = array_data_left_x[i];
                int y2 = array_data_right_y[i];
                int y1 = array_data_left_y[i];

                auto interpolated_points = linearInterpolation(x1, y1, x2, y2, num_points); // first x , second y
                int_vector_x.insert(int_vector_x.end(), interpolated_points.first.begin(), interpolated_points.first.end());
                int_vector_y.insert(int_vector_y.end(), interpolated_points.second.begin(), interpolated_points.second.end());
            }


            occupancy_grid_msg.header.frame_id = "map";  // Doldurmak istediğiniz frame_id'yi belirtin.
            occupancy_grid_msg.info.width = width;
            occupancy_grid_msg.info.height = height;
            occupancy_grid_msg.info.origin.position.x = min_x;
            occupancy_grid_msg.info.origin.position.y = min_y;
            occupancy_grid_msg.info.origin.position.z = 0.0;
            occupancy_grid_msg.info.resolution = 1.0; 
            occupancy_grid_msg.data = createMatrixWithMod( int_vector_x , int_vector_y , width , height , min_x , min_y);
            first_ = false;
            double x_i = vehicle_odom_->pose.pose.position.x - min_x;
            double y_i = vehicle_odom_->pose.pose.position.y - min_y;
            double x_e = goal_msg_.pose.position.x - min_x;
            double y_e = goal_msg_.pose.position.y - min_y;
            grid_msg_.data = {static_cast<double>(width) , static_cast<double>(height),x_i,y_i,x_e,y_e,static_cast<double>(min_x),static_cast<double>(min_y)};

            prev_count_ = count_;
        }
        obsx_publisher_->publish(x_msg_);
        obsy_publisher_->publish(y_msg_);
        grid_size_publisher->publish(grid_msg_);
        occupancy_grid_publisher_->publish(occupancy_grid_msg);

    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        vehicle_odom_ = msg;
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr t_goal_msg) {
        goal_msg_ = *t_goal_msg;
        flag2=true;
        count_ ++;
    }

    void initialCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr t_init_msg) {
        init_msg_ = *t_init_msg;
        // flag1=true;
    }

    void float64MultiArrayCallback(const std_msgs::msg::Float64MultiArray::SharedPtr t_msg)
    {
        if(first)
        {
            for(size_t i = 0 ; i < t_msg->data.size()/4 ; i++)
            {
                array_data_left_x.push_back(round(t_msg->data[4*i]));
                array_data_left_y.push_back(round(t_msg->data[4*i+1]));
                array_data_right_x.push_back(round(t_msg->data[4*i+2]));
                array_data_right_y.push_back(round(t_msg->data[4*i+3]));
            }
            first = false;
        }
    }
    
    std::vector<int8_t> createMatrixWithMod(std::vector<int32_t>& int_vector_x, std::vector<int32_t>& int_vector_y, int& rows, int& columns, int& min_x, int& min_y) 
    {
        std::cout << int_vector_y.size()<< std::endl;
        std::vector<int8_t> matrix_data(rows * columns, 100);
        std::vector<std::vector<int>> dil_matrix(rows, std::vector<int>(columns, 0));
        for (size_t i = 0; i < int_vector_x.size(); ++i) 
        {
            int32_t x = int_vector_x[i] - min_x;
            int32_t y = int_vector_y[i] - min_y;

            if (x >= 0 && x < rows && y >= 0 && y < columns) {
                matrix_data[(y * rows) + x] = 0;
            }
        }

        for( int i = 0 ; i < columns ; i++ )
        {
            for( int j = 0 ; j < rows ; j++ )
            {
                if(matrix_data[(i * rows) + j]==100)
                {
                    x_msg_.data.push_back(j);
                    y_msg_.data.push_back(i);
                }
                else
                {                    
                    dil_matrix[j][i]=1;
                }
            }
        }
        dil_matrix=applyDilation(dil_matrix);

        std::vector<int8_t> dil ;
        for( int i = 0 ; i < columns ; i++ )
        {
            for( int j = 0 ; j < rows ; j++ )
            {
                if (dil_matrix[j][i] == 1)
                {
                    dil.push_back(0);
                }
                else
                {
                    dil.push_back(100);
                }
            }
        }
        return dil;
    }
    
    double getDistance(int &x1,int &x2,int &y1,int &y2)
    {
        return std::sqrt(std::pow(x1-x2, 2) + std::pow(y1 - y2, 2));
    }

    double getDistance(const int x1, const int x2, const int y1, const int y2) 
    {
        return std::sqrt(std::pow(x1-x2, 2) + std::pow(y1 - y2, 2));
    }

    std::pair<std::vector<int32_t>, std::vector<int32_t>> linearInterpolation(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int num_points) 
    {
        std::vector<int32_t> int_vector_x;
        std::vector<int32_t> int_vector_y;

        for (int k = 0; k < num_points; ++k) {
            double t = static_cast<double>(k) / (num_points - 1);
            int32_t x = static_cast<int32_t>(round(x1 + t * (x2 - x1)));
            int32_t y = static_cast<int32_t>(round(y1 + t * (y2 - y1)));
            int_vector_x.push_back(x);
            int_vector_y.push_back(y);
        }

        return std::make_pair(int_vector_x, int_vector_y);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr osm_sub_; 
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_subscriber_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr obsx_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr obsy_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr grid_size_publisher;
    std_msgs::msg::Float64MultiArray x_msg_;
    std_msgs::msg::Float64MultiArray y_msg_;
    std_msgs::msg::Float64MultiArray grid_msg_;
    geometry_msgs::msg::PoseStamped goal_msg_;
    geometry_msgs::msg::PoseWithCovarianceStamped init_msg_;
    bool flag1 =false;
    bool flag2 =false;
    int count_{0};
    int prev_count_{0};


    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::OccupancyGrid occupancy_grid_msg;
    std::shared_ptr<nav_msgs::msg::Odometry> vehicle_odom_;
    std::vector<double> array_data_left_x;
    std::vector<double> array_data_left_y;
    std::vector<double> array_data_right_x;
    std::vector<double> array_data_right_y;
    bool first_{true};
    bool first{true};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyGridPublisher>());
    rclcpp::shutdown();
    return 0;
}
    