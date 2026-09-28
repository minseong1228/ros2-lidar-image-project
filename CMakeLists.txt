cmake_minimum_required(VERSION 3.8)
project(embedded_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetROI.srv"
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(image_publisher
  src/image_publisher.cpp
  src/image_utils.cpp
)
ament_target_dependencies(image_publisher
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV
)
rosidl_target_interfaces(image_publisher
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(roi_selector src/roi_selector.cpp)
ament_target_dependencies(roi_selector
  rclcpp
)
rosidl_target_interfaces(roi_selector
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  image_publisher
  roi_selector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
