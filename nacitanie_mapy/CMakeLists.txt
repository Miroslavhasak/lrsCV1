cmake_minimum_required(VERSION 3.8)
project(pcd_map_loader)

# pre vypisy a varovania
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(pcd_map_loader_node src/pcd_map_loader.cpp)
ament_target_dependencies(pcd_map_loader_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(pcd_map_loader_node
  ${PCL_LIBRARIES}
)

install(TARGETS
  pcd_map_loader_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

