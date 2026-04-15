cmake_minimum_required(VERSION 3.8)
project(omnimagnet_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_library(COMEDI_LIB comedi PATHS /usr/lib)

add_executable(omnimagnet_driver 
  src/omnimagnet_driver.cpp
  src/omnimagnet.cpp
  src/type.cpp)
ament_target_dependencies(omnimagnet_driver rclcpp)
target_link_libraries(omnimagnet_driver ${COMEDI_LIB})

install(TARGETS
  omnimagnet_driver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
