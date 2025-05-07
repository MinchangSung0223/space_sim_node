cmake_minimum_required(VERSION 3.14)
project(space_sim_node)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include paths
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Sources
add_executable(space_sim_node src/SpaceSimNode.cpp)
target_link_libraries(space_sim_node SpaceSim)
ament_target_dependencies(space_sim_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

install(TARGETS space_sim_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()