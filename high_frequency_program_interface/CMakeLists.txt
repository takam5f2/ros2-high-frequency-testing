cmake_minimum_required(VERSION 3.8)
project(high_frequency_program_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(high_frequency_program_interface
  "srv/Frequency.srv"
)

ament_export_dependencies(rosidl_default_generators)

install(
  DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

ament_package()