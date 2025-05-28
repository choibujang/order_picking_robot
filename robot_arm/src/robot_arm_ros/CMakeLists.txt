cmake_minimum_required(VERSION 3.8)
project(robot_arm_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ros_interfaces REQUIRED)
find_package(OrbbecSDK REQUIRED)
find_package(robot_arm_controllers REQUIRED)
find_package(kdl_parser REQUIRED)


add_executable(robot_arm_node src/robot_arm_node.cpp)

target_include_directories(robot_arm_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(robot_arm_node 
  PUBLIC 
    c_std_99 cxx_std_17)  # Require C99 and C++17


ament_target_dependencies(robot_arm_node
  PUBLIC
    "ros_interfaces"
    "rclcpp"
    "rclcpp_action"
    "rclcpp_components"
    "robot_arm_controllers"
    "kdl_parser")

install(TARGETS robot_arm_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
