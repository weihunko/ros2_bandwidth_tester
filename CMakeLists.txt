cmake_minimum_required(VERSION 3.5)
project(ros2_bandwidth_tester)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(bandwidth_tester src/bandwidth_tester.cpp)

target_include_directories(bandwidth_tester PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${rclcpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
)

target_link_libraries(bandwidth_tester PUBLIC
    ${rclcpp_LIBRARIES}
    ${std_msgs_LIBRARIES}
)

add_executable(bandwidth_tester_apex src/bandwidth_tester_apex.cpp)

target_include_directories(bandwidth_tester_apex PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${rclcpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
)

target_link_libraries(bandwidth_tester_apex PUBLIC
    ${rclcpp_LIBRARIES}
    ${std_msgs_LIBRARIES}
)

install(TARGETS
  bandwidth_tester
  bandwidth_tester_apex
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
