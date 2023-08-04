<h1 align="center">map_visualizer package</h1>

# :dart: About Package #
This package provides the visualize the map in ROS2 using MarkerArray and Occupancygrid map types.
There is two node <a href="#A---osm_visualizer-node">osm_visualizer node</a> and <a href="#B---occupancy_pub-node">occupancy_pub node</a> in this package.

The following tools were used in this project:

- [ROS](https://www.ros.org/) Tested on ROS2 Foxy distro.

## :white_check_mark: Package Requirements ##
Before starting :checkered_flag:, you need to have ROS2(Foxy) and install some ros2 packages.

`sudo rosdep init`

`rosdep update`

`rosdep install --from-paths src -y --ignore-src`

`sudo apt-get install ros-<distro>-lanelet2`

E.g: `sudo apt-get install ros-foxy-lanelet2`

## :heavy_check_mark: To Do ##

### osm_visualizer node
* For clean code, Separate filling float64multiarray and markerarray from each other and write a new node for float64multiarray 

### occupancy_pub node
* For clean code, Abort initial pose callback take the initial position from odometry and keep it unless change the goal position 

* Separate Compilation Technique, Separate the single file into implemantation declaration and main files 

* Read subscriber topic names in param file

# A - osm_visualizer node

## :dart: 1. About ##
osm_visualizer node visualizes the .osm(OpenStreetMap) file into RViz MarkerArray. 

## :heavy_plus_sign: :heavy_minus_sign: 2. Input/Output ##

Input:
* osm(OpenStreetMap) file

Output:
* /hd_map (visualization_msgs/msg/marker_array) : visualization messages for RViz
* /array (std_msgs/msg/float64_multi_array) : the left and right bounds points of the map lanelet layers for occupancy_pub node

### Parameters

| Name                   | Type        | Description                                                                                         | Default value           |
| :--------------------- | :---------- | :-------------------------------------------------------------------------------------------------  | :---------------------- |
| map_path               | std::string | lanelet2 map path                                                                                   | change on launch file   |
| enable_inc_path_points | bool        | flag to increment points of left and rigth boundry's linestrings                                    | true                    |
| interval               | std::string | interval to increment left and right boundry points with linear interpolation for occupancygrid map | 0.5                     |

## :infinity: 3. Pseudo Code
* Read osm file
* loop the lanelet layers in osm file
* get the left and right boundry of lanelet layers
* again loop the points of left and right boundry
* push_back boundry points into the marker points and float64multiarray with 
* publish marker array(for visualization) and float64multiarray(for occupancygrid node)

## :white_check_mark: 4. Requirements ##
Before starting :checkered_flag:, you need to have ros2 lanelet library.

## :x: 5. Dependencies ##
 - (Bağlı olan algoritmalar github konumlarından declaration kodları çekildikten sonra include dosyaları burada olmalıdır.)

### include
* None

### src
* None

## :checkered_flag: 6. How to Run ##

* `ros2 run map_visualizer osm_visualizer --ros-args -p map_path:=path/to/map.osm`

or

* `ros2 launch map_visualizer osm_visualizer.launch.xml` you need to change map path

or

* `ros2 launch map_visualizer osm_visualizer.launch.py` you need to change map path

# B - occupancy_pub node

## :dart: 1. About ##
occupancy_pub node turn the osm data provided by osm_visualizer node into nav_msgs/msg/occupancy_grid. 

## :heavy_plus_sign: :heavy_minus_sign: 2. Input/Output

Input:
* /array (std_msgs/msg/float64_multi_array) : the left and right bounds points of the map lanelet layers
* /initialpose (geometry_msgs/msg/pose_with_covariance_stamped) : RViz initial pose message to calculate initial location on grid
* /goal_pose (geometry_msgs/msg/pose_stamped) : RViz goal pose message to calculate goal location on grid

Output:
* /occupancy_grid (nav_msgs/msg/occupancy_grid) : occupancy grid messages for RViz
* /obsx (std_msgs/msg/float64_multi_array) : x index of obstacles on grid for A* package
* /obsy (std_msgs/msg/float64_multi_array) : y index of obstacles on grid for A* package
* /gridsize (std_msgs/msg/float64_multi_array) : gridsize information message for A* package

### Parameters
* To Do !!!

## :infinity: 3. Pseudo Code
* subscribe callback data
* find max and min x y to find width height of occupacygrid map
* create std::vector<int8_t> matrix_data(rows * columns, 100) all obstacles
* to find drivable area, fill the area between left and right bounds of lane with 0 and change the values of this rows, columns
* publish the required data

## :white_check_mark: 4. Requirements ##
Before starting :checkered_flag:, you need to run map_visualizer node.


## :x: 5. Dependencies ##
 - (Bağlı olan algoritmalar github konumlarından declaration kodları çekildikten sonra include dosyaları burada olmalıdır.)

### include
*  include "../include/osmVisualizer/dilation.h"

### src
* None

## :checkered_flag: 6. How to Run ##

* `ros2 run map_visualizer occupancy_pub`


# References
* https://github.com/fzi-forschungszentrum-informatik/Lanelet2

## :memo: License

This project is under license from MIT. For more details, see the [LICENSE](LICENSE) file.

Made with by <a href="https://github.com/mucahitayhan" target="_blank">Mücahit Ayhan</a> and <a href="https://github.com/GokmenAtakanT" target="_blank">Gökmen Atakan Türkmen</a>.
&#xa0;

<a href="#top">Back to top</a>