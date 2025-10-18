cmake_minimum_required(VERSION 3.8)
project(prehladavaci_algoritmus)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)


# pridaj všetky cpp súbory, ktoré tvoria planner
add_executable(planner_node
  src/plannerAlgorithm.cpp
  src/voxel_map.cpp
  src/a_star.cpp
)

ament_target_dependencies(planner_node
  rclcpp sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_ros
)

# drone controller node
add_executable(drone_controller src/drone_controller.cpp)
ament_target_dependencies(drone_controller rclcpp geometry_msgs tf2)

install(TARGETS
  planner_node
  drone_controller
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)


ament_package()

