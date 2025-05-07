cmake_minimum_required(VERSION 3.5)
project(space_sim_node)

# ── ROS 2 / 외부 의존성 ───────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 17)

# ── IMPORTED library : libSpaceSim.so 하나만 사용 ────────────
set(SPACE_SIM_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_library(SpaceSim SHARED IMPORTED)
set_target_properties(SpaceSim PROPERTIES
  IMPORTED_LOCATION ${SPACE_SIM_LIB_DIR}/libSpaceSim.so
  INTERFACE_INCLUDE_DIRECTORIES ${PROJECT_SOURCE_DIR}/include
)

# ── Executable ────────────────────────────────────────────────
add_executable(space_sim_node src/SpaceSimNode.cpp)

target_include_directories(space_sim_node PRIVATE
  ${PROJECT_SOURCE_DIR}/include  # SpaceSim.hpp 등
)

target_link_libraries(space_sim_node SpaceSim Eigen3::Eigen)

# 런타임 rpath : <exe_dir>/../lib 에서 .so 탐색
# ($ORIGIN 은 실행파일 위치)
set_target_properties(space_sim_node PROPERTIES
  INSTALL_RPATH "\$ORIGIN/../lib"
  BUILD_RPATH   "${CMAKE_CURRENT_BINARY_DIR}/lib"
)

ament_target_dependencies(space_sim_node
  rclcpp geometry_msgs sensor_msgs std_msgs Eigen3)

# ── 설치 ──────────────────────────────────────────────────────
install(TARGETS space_sim_node DESTINATION lib/${PROJECT_NAME})

#  libSpaceSim.so → install/<pkg>/lib/ 로 복사
install(FILES ${SPACE_SIM_LIB_DIR}/libSpaceSim.so DESTINATION lib/${PROJECT_NAME})

# ── 헤더 설치 (SpaceSim.hpp) ─────────────────────────────────
install(DIRECTORY include/ DESTINATION include/)

ament_package()