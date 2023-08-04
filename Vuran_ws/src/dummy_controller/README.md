# Dummy Controller

## Özet

Path Planning algoritmalarini test etmek icin gelistirilmis dummy controller.
Arac modeli olarak **Kinematic Bicycle Model kullanir**.
**PID** kontrol ile throotle kontrolu ve **Pure Pursuit** ile de steer kontrolu amaclanmistir. 

## Input & Output

### Input Topics

| Name                                  | Type                                  | Description                                               |
| --------------------------------------| --------------------------------------| --------------------------------------------------------- |
| `/carla/ego_vehicle/waypoints`        | nav_msgs::msg::Path                   | Carla simulation waypoint path                            |
| `/carla/ego_vehicle/odometry`         | nav_msgs/msg/Odometry                 | Carla simulation ego vehicle odometry                     |
| `/carla/ego_vehicle/vehicle_status`   | carla_msgs/msg/CarlaEgoVehicleStatus  | Carla simulation ego vehicle status                       |

### Output Topics

| Name                                     | Type                                    | Description                                               |
| -----------------------------------------| ----------------------------------------| --------------------------------------------------------- |
| `/carla/ego_vehicle/vehicle_control_cmd` | carla_msgs::msg::CarlaEgoVehicleControl | Ego vehicle control over steer, throttle and brake        |

## Kurulum

```bash
cd colcon_ws/src
git clone <url>
cd ..
colcon build --packages-select dummy_controller
```

## Dependencies

- carla_msgs

## Nasıl Çalıştırılır
```bash
ros2 launch dummy_controller dummy_controller.py
```

## Referanslar
* [Algorithms for Automated Driving](https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/ControlOverview.html)