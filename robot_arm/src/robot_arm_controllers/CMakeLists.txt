message(STATUS "==== CMakeLists.txt is being read ====")

cmake_minimum_required(VERSION 3.8)
project(robot_arm_controllers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OrbbecSDK REQUIRED)
find_package(OpenCV REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_library(I2C_LIB NAMES i2c PATHS /usr/lib /usr/local/lib)

add_library(robot_arm_controllers SHARED
    src/arm/arm_controller.cpp
    src/arm/I2CPeripheral.cpp
    src/arm/pca9685_comm.cpp
    src/cam/cam_controller.cpp
    src/net/net_controller.cpp
)

target_include_directories(robot_arm_controllers
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(robot_arm_controllers
  PUBLIC
    ${I2C_LIB} 
    OrbbecSDK::OrbbecSDK
    ${OpenCV_LIBS}

)

ament_target_dependencies(robot_arm_controllers
  PUBLIC
    ament_index_cpp
    orocos_kdl
    kdl_parser
)

add_executable(motor_test test/motor_test.cpp)
target_link_libraries(motor_test PUBLIC robot_arm_controllers)

add_executable(send_img_test test/send_img_test.cpp)
target_link_libraries(send_img_test PUBLIC robot_arm_controllers)

add_executable(depthmap_test test/depthmap_test.cpp)
target_link_libraries(depthmap_test PUBLIC robot_arm_controllers)

install(
  TARGETS robot_arm_controllers
  EXPORT export_robot_arm_controllers
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/ 
  DESTINATION include/
)

install(
  EXPORT export_robot_arm_controllers
  FILE robot_arm_controllersTargets.cmake
  NAMESPACE robot_arm_controllers::
  DESTINATION share/robot_arm_controllers/cmake
)

ament_export_include_directories(include)
ament_export_libraries(robot_arm_controllers)
ament_export_targets(export_robot_arm_controllers)

ament_package()