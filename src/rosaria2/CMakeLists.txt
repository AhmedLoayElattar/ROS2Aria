cmake_minimum_required(VERSION 3.8)
project(rosaria2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# ───────────────────────────────────────────────
# Find ROS2 dependencies
# ───────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Threads REQUIRED)

# ───────────────────────────────────────────────
# Build AriaCoda as a static library
# ───────────────────────────────────────────────
set(ARIACODA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/AriaCoda")

# Collect all AriaCoda source files, excluding Windows-specific ones
file(GLOB ARIACODA_SOURCES "${ARIACODA_DIR}/src/*.cpp")
# Remove Windows-specific sources on Linux/Mac
list(FILTER ARIACODA_SOURCES EXCLUDE REGEX ".*_WIN\\.cpp$")

add_library(aria_static STATIC ${ARIACODA_SOURCES})
target_include_directories(aria_static PUBLIC
  "${ARIACODA_DIR}/include"
)
target_compile_definitions(aria_static PRIVATE
  -DARIA_STATIC
)
target_compile_options(aria_static PRIVATE
  -fPIC
  -w  # Suppress warnings from third-party code
)
# AriaCoda needs pthreads, dl, rt on Linux
target_link_libraries(aria_static PUBLIC
  Threads::Threads
  ${CMAKE_DL_LIBS}
  rt
)

# ───────────────────────────────────────────────
# Build the ROS2 node
# ───────────────────────────────────────────────
add_executable(rosaria2_node
  src/rosaria2_node.cpp
  src/laser_publisher.cpp
)

target_include_directories(rosaria2_node PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${ARIACODA_DIR}/include"
)

target_compile_options(rosaria2_node PRIVATE -fPIC)

ament_target_dependencies(rosaria2_node
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(rosaria2_node
  aria_static
)

# ───────────────────────────────────────────────
# Install
# ───────────────────────────────────────────────
install(TARGETS rosaria2_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()
