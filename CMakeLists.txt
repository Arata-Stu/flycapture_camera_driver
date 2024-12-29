cmake_minimum_required(VERSION 3.8)
project(flycapture_camera_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)


include_directories(/usr/include/flycapture)
link_directories(/usr/lib)

add_executable(
  camera_recorder
  src/camera_recorder.cpp
)

ament_target_dependencies(camera_recorder rclcpp OpenCV std_msgs)
TARGET_LINK_LIBRARIES(camera_recorder /usr/lib/libflycapture.so -lpthread -lm)

install(TARGETS
  camera_recorder
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})  

ament_package()
