
#!/bin/bash

# Run the CarlaUE4.sh script in /opt/carla-simulator
gnome-terminal --tab -- bash -c "cd /opt/carla-simulator; ./CarlaUE4.sh"

# Wait for Carla to start up
sleep 15

# Launch the carla_ros_bridge with example ego vehicle
gnome-terminal --tab -- bash -c "cd ~/carla-ros-bridge; ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py"

# Wait for the bridge to start up
sleep 10

# Run map_visualizer occupancy_pub node
gnome-terminal --tab -- bash -c "cd ~/Vuran_ws; ros2 run map_visualizer occupancy_pub"

# Run map_visualizer osm_visualizer node
gnome-terminal --tab -- bash -c "cd ~/Vuran_ws; ros2 run map_visualizer osm_visualizer"

gnome-terminal --tab -- bash -c "rviz2"
# Run vuran_path_planning main node
#gnome-terminal --tab -- bash -c "cd ~/Vuran_ws; ros2 run vuran_path_planning main"

# Keep the script running to keep the tabs open
read -p "Press Enter to close the tabs..."

