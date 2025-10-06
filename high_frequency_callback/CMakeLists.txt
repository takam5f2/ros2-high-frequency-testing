cmake_minimum_required(VERSION 3.8)
project(high_frequency_callback)

# ROS Humble向けにCXX標準を設定
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(high_frequency_program_interface REQUIRED)

# ヘッダーファイルのインクルードパスを追加
include_directories(include)


# 実行可能ファイルの追加
add_executable(timer_node src/main.cpp)
ament_target_dependencies(timer_node rclcpp high_frequency_program_interface)

add_library(timer SHARED src/timer.cpp)
ament_target_dependencies(timer rclcpp high_frequency_program_interface)


target_link_libraries(timer_node timer)

# ノードのインストール
install(TARGETS
  timer_node
  DESTINATION lib/${PROJECT_NAME}
)

# ヘッダーファイルのインストール
install(DIRECTORY
  include/
  DESTINATION include
)


ament_package()
