cmake_minimum_required(VERSION 3.8)
project(festival_ur_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(festival_ur_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)


include_directories(
  include
)


set(DEPENDENCIES
    "Eigen3"
    "std_msgs"
    "rclcpp"
    "tf2"
    "tf2_ros"
    "festival_ur_interfaces"
    "moveit_ros_planning_interface"
)

set(EXECUTABLE_NAME "festival_ur_core")

add_executable(${EXECUTABLE_NAME} src/core_main.cpp)
target_link_libraries(${EXECUTABLE_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${EXECUTABLE_NAME} ${DEPENDENCIES})




ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
launch/festival_ur_core_node.launch.py
launch/test.launch.py
DESTINATION share/${PROJECT_NAME}/launch # Update the destination
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)


install(DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()