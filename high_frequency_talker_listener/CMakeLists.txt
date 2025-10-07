cmake_minimum_required(VERSION 3.8)
project(high_frequency_talker_listener)

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

# ヘッダーファイルのインクルードパスを追加
include_directories(include)


# looping_talker
add_library(looping_talker SHARED src/looping_talker.cpp)
ament_target_dependencies(looping_talker rclcpp std_msgs)

add_executable(looping_talker_node src/looping_talker_node.cpp)
ament_target_dependencies(looping_talker_node rclcpp)
target_link_libraries(looping_talker_node looping_talker)

# event_driven_listener
add_library(event_driven_listener SHARED src/event_driven_listener.cpp)
ament_target_dependencies(event_driven_listener rclcpp std_msgs)

add_executable(event_driven_listener_node src/event_driven_listener_node.cpp)
ament_target_dependencies(event_driven_listener_node rclcpp)
target_link_libraries(event_driven_listener_node event_driven_listener)

# timer_drinven_listener
add_library(timer_driven_listener SHARED src/timer_driven_listener.cpp)
ament_target_dependencies(timer_driven_listener rclcpp std_msgs)

add_executable(timer_driven_listener_node src/timer_driven_listener_node.cpp)
ament_target_dependencies(timer_driven_listener_node rclcpp)
target_link_libraries(timer_driven_listener_node timer_driven_listener)

# busy_looping_listener
add_library(busy_looping_listener SHARED src/busy_looping_listener.cpp)
ament_target_dependencies(busy_looping_listener rclcpp std_msgs)
add_executable(busy_looping_listener_node src/busy_looping_listener_node.cpp)
ament_target_dependencies(busy_looping_listener_node rclcpp)
target_link_libraries(busy_looping_listener_node busy_looping_listener)

# ノードのインストール
install(TARGETS
  looping_talker
  looping_talker_node
  event_driven_listener
  event_driven_listener_node
  timer_driven_listener
  timer_driven_listener_node
  busy_looping_listener
  busy_looping_listener_node
  DESTINATION lib/${PROJECT_NAME})


# ヘッダーファイルのインストール
install(DIRECTORY
  include/
  DESTINATION include
)


ament_package()
