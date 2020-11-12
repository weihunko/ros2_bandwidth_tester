cmake_minimum_required(VERSION 3.5)
project(ros2_bandwidth_tester)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
    "idl/TestString.idl")

ament_auto_add_executable(bandwidth_tester src/bandwidth_tester.cpp)

rosidl_target_interfaces(bandwidth_tester ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_auto_package()
