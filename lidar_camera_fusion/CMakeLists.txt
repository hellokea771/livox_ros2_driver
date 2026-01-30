cmake_minimum_required(VERSION 3.8)
project(lidar_camera_fusion)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

add_executable(fusion_node src/fusion_node.cpp)

target_include_directories(fusion_node PUBLIC
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

target_link_libraries(fusion_node
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)

ament_target_dependencies(fusion_node
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
  pcl_conversions
)

install(TARGETS fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()