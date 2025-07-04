cmake_minimum_required(VERSION 3.8)
project(robot_arm_core)

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

add_library(robot_arm_core SHARED
    src/controllers/arm_controller.cpp
    src/controllers/I2CPeripheral.cpp
    src/controllers/pca9685_comm.cpp
    src/controllers/cam_controller.cpp
    src/controllers/net_controller.cpp
    src/arm_workflow_manager.cpp
)

target_include_directories(robot_arm_core
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(robot_arm_core
  PUBLIC
    ${I2C_LIB} 
    OrbbecSDK::OrbbecSDK
    ${OpenCV_LIBS}

)

ament_target_dependencies(robot_arm_core
  PUBLIC
    ament_index_cpp
    orocos_kdl
    kdl_parser
)

add_executable(motor_test test/motor_test.cpp)
target_link_libraries(motor_test PUBLIC robot_arm_core)

add_executable(send_img_test test/send_img_test.cpp)
target_link_libraries(send_img_test PUBLIC robot_arm_core)

add_executable(depthmap_test test/depthmap_test.cpp)
target_link_libraries(depthmap_test PUBLIC robot_arm_core)

install(
  TARGETS robot_arm_core
  EXPORT robot_arm_core
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
  EXPORT robot_arm_core
  FILE robot_arm_coreTargets.cmake
  NAMESPACE robot_arm_core::
  DESTINATION share/robot_arm_core/cmake
)

ament_export_include_directories(include)
ament_export_libraries(robot_arm_core)
ament_export_targets(robot_arm_core)

ament_package()