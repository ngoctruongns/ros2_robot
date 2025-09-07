cmake_minimum_required(VERSION 3.8)
project(cpp_srvcli)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(tutorial_interfaces REQUIRED)

add_executable(add_two_server src/add_two_ints_server.cpp)
ament_target_dependencies(add_two_server rclcpp example_interfaces)

add_executable(add_three_server src/add_three_ints_server.cpp)
ament_target_dependencies(add_three_server rclcpp tutorial_interfaces)

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client rclcpp example_interfaces tutorial_interfaces)

install(TARGETS
    add_two_server
    add_three_server
    client
    DESTINATION lib/${PROJECT_NAME})

ament_package()
