cmake_minimum_required(VERSION 3.8)
project(g_mov_cam)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS highgui imgcodecs imgproc videoio)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

if(DEFINED ENV{ROS_DISTRO})
  message(STATUS "Building for ROS 2: $ENV{ROS_DISTRO}")
  if("$ENV{ROS_DISTRO}" STREQUAL "jazzy")
    add_compile_definitions(ROS2_JAZZY)
  endif()
endif()

add_executable(custom_image_transport src/custom_image_transport.cpp)
ament_target_dependencies(custom_image_transport
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)


# Install executables
install(
  TARGETS custom_image_transport
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


ament_export_include_directories(include)
ament_export_dependencies(cv_bridge image_transport rclcpp sensor_msgs std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
