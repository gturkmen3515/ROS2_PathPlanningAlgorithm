#!/bin/bash

# Function to run a command in a new terminal window
function run_in_terminal() {
    gnome-terminal -- bash -c "$1; exec bash"
}

# Run the CarlaUE4.sh script in /opt/carla-simulator
cd /opt/carla-simulator
run_in_terminal "./CarlaUE4.sh"

# Wait for Carla to start up
sleep 15

# Launch the carla_ros_bridge with example ego vehicle

cd ~/carla-ros-bridge
run_in_terminal "ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py"

# Wait for the bridge to start up
sleep 10

# Run map_visualizer occupancy_pub node
cd ~/Vuran_ws
run_in_terminal "ros2 run map_visualizer occupancy_pub"

# Run map_visualizer osm_visualizer node
run_in_terminal "ros2 run map_visualizer osm_visualizer"

run_in_terminal "rviz2"

# Run vuran_path_planning main node
#run_in_terminal "ros2 run vuran_path_planning main"
#run_in_terminal "ros2 run dummy_controller dummy_controller_node"


