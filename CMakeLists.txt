cmake_minimum_required(VERSION 3.8)
project(festival_ur_core_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(festival_ur_interfaces REQUIRED)

add_executable(festival_ur_core src/festival_ur_core.cpp)
target_include_directories(festival_ur_core PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_compile_features(festival_ur_core PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
ament_target_dependencies(
    festival_ur_core
    std_msgs
    rclcpp
    tf2
    tf2_ros
    festival_ur_interfaces
"moveit_ros_planning_interface"
"rclcpp"
)



ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
launch/festival_ur_core_node.launch.py
launch/test.launch.py
DESTINATION share/${PROJECT_NAME}/launch # Update the destination
)

install(TARGETS festival_ur_core
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME} # Update the destination
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
set(ament_cmake_copyright_FOUND TRUE)
set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()
endif()

ament_package()