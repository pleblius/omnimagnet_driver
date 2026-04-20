cmake_minimum_required(VERSION 3.8)
project(omnimagnet_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ErrorMessage.msg"
  "msg/Vector3.msg"
  "msg/FinishedMessage.msg"
  "srv/DriverReset.srv"
  "srv/MultiMagnetConstant.srv"
  "srv/MultiMagnetRotation.srv"
  "srv/SingleMagnetConstant.srv"
  "srv/SingleMagnetRotation.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
