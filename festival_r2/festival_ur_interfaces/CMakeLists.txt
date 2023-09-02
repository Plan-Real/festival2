cmake_minimum_required(VERSION 3.5)
project(festival_ur_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Purpose.msg"
  "msg/Point2D.msg"
  "msg/Vector2.msg"
  "msg/Pose2D.msg"
  "msg/BoundingBox2D.msg"
  "msg/BoundingBox3D.msg"
  "msg/Mask.msg"
  "msg/KeyPoint2D.msg"
  "msg/KeyPoint2DArray.msg"
  "msg/KeyPoint3D.msg"
  "msg/KeyPoint3DArray.msg"
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "srv/MoveJoint.srv"
  DEPENDENCIES std_msgs geometry_msgs# Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)


ament_package()
