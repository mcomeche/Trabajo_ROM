cmake_minimum_required(VERSION 3.8)
project(practica9_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)

# Declare your executable
add_executable(zone_navigator src/zone_navigator.cpp)

# Link dependencies to your executable
ament_target_dependencies(zone_navigator
  rclcpp
  geometry_msgs
  nav2_msgs
  rclcpp_action
)


install(TARGETS zone_navigator
  DESTINATION lib/${PROJECT_NAME}  # Esto creará lib/practica9_pkg/
)


install(DIRECTORY launch param
DESTINATION share/${PROJECT_NAME})

ament_package()
