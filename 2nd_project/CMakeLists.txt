cmake_minimum_required(VERSION 3.8)
project(cpp_swarm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Nájdenie závislostí
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Vytvorenie spustiteľného súboru (executable)
add_executable(swarm_node src/swarm_node.cpp)

# Linkovanie knižníc
ament_target_dependencies(swarm_node
  rclcpp
  mavros_msgs
  geometry_msgs
  sensor_msgs
)

# Inštalácia (kam sa má súbor skopírovať po builde)
install(TARGETS
  swarm_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
