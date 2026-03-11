cmake_minimum_required(VERSION 3.15)
project(ros2_voxelsdk)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# The base VoxelSDK might already be available in the upper scope, 
# but we find it here just in case this is built as a standalone ROS module
find_package(Voxel 0.6.11 REQUIRED)

add_executable(voxelsdk_node src/voxelsdk_node.cpp)

target_include_directories(voxelsdk_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Ament dependencies
ament_target_dependencies(voxelsdk_node
  rclcpp
  sensor_msgs
  std_msgs
)

# Link against the Voxel library
target_link_libraries(voxelsdk_node PUBLIC ${VOXEL_LIBRARIES})
target_include_directories(voxelsdk_node PUBLIC ${VOXEL_INCLUDE_DIRS})

install(TARGETS voxelsdk_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
