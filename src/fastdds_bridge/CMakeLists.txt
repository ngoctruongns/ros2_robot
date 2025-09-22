cmake_minimum_required(VERSION 3.8)
project(fastdds_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Fast DDS dependencies
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)

include_directories(
  include
)

#  Create library for HelloWorld

file(GLOB DDS_HELLOWORLD_SOURCES_CXX "src/*.cxx")
add_library(HelloWorld_lib ${DDS_HELLOWORLD_SOURCES_CXX})
target_link_libraries(HelloWorld_lib fastcdr fastrtps)

# Generate Fast DDS Node
add_executable(hello_sub src/HelloSubMain.cpp)
target_link_libraries(hello_sub fastrtps fastcdr HelloWorld_lib)

# Generate the ROS2 node
add_executable(hello_bridge_node src/main.cpp)
ament_target_dependencies(hello_bridge_node
  rclcpp
  std_msgs
)

# Link the Fast DDS libraries
target_link_libraries(hello_bridge_node fastrtps fastcdr HelloWorld_lib)

# Install to project
install(TARGETS
  hello_sub
  hello_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
