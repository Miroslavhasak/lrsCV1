cmake_minimum_required(VERSION 3.8)

project(csv_reader)



find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)



add_executable(csv_reader_node src/csv_reader_node.cpp)

ament_target_dependencies(csv_reader_node rclcpp)



install(TARGETS

  csv_reader_node

  DESTINATION lib/${PROJECT_NAME})



ament_package()
