cmake_minimum_required(VERSION 3.8)
project(festival_ur_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(festival_ur_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)


include_directories(
  include
)

add_library(${PROJECT_NAME}_lib
  "src/festival.cpp"
)

set(DEPENDENCIES
    "Eigen3"
    "std_msgs"
    "sensor_msgs"
    "std_srvs"
    "rclcpp"
    "tf2"
    "tf2_ros"
    "festival_ur_interfaces"
)

set(EXECUTABLE_NAME "festival_ur_core")

target_link_libraries(${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

add_executable(${EXECUTABLE_NAME} src/core_main.cpp)
target_link_libraries(${EXECUTABLE_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${EXECUTABLE_NAME} ${DEPENDENCIES})





install(PROGRAMS
  launch/festival_ur_core.launch.py
  launch/test.launch.py
  DESTINATION share/${PROJECT_NAME}/launch # Update the destination
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)


install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()
