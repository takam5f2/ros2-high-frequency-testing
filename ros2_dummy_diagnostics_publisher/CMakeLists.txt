cmake_minimum_required(VERSION 3.8)
project(ros2_dummy_diagnostics_publisher)

# ROS Humble向けにCXX標準を設定
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

# ヘッダーファイルのインクルードパスを追加
include_directories(include)


# 
add_library(ros2_dummy_diagnostics_publisher SHARED src/ros2_dummy_diagnostics_publisher.cpp)
ament_target_dependencies(ros2_dummy_diagnostics_publisher rclcpp diagnostic_updater)

add_executable(ros2_dummy_diagnostics_publisher_node src/ros2_dummy_diagnostics_publisher_node.cpp)
ament_target_dependencies(ros2_dummy_diagnostics_publisher_node rclcpp diagnostic_updater)
target_link_libraries(ros2_dummy_diagnostics_publisher_node ros2_dummy_diagnostics_publisher)


# ノードのインストール
install(TARGETS
  ros2_dummy_diagnostics_publisher_node
  DESTINATION lib/${PROJECT_NAME})


# ヘッダーファイルのインストール
install(DIRECTORY
  include/
  DESTINATION include
)


ament_package()
