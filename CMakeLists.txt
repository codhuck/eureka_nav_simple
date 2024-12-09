cmake_minimum_required(VERSION 3.16)
project(eureka_nav_simple LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  sensor_msgs
  geometry_msgs
)

foreach(Dependency IN LISTS THIS_PACKAGE_INCLUDE_DEPENDS)
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include/)

add_executable(eureka_simple_cv
  src/nav_simple.cpp
)

ament_target_dependencies(eureka_simple_cv
  rclcpp
  sensor_msgs
  geometry_msgs
)

# INSTALL
install(TARGETS 
  eureka_simple_cv
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()