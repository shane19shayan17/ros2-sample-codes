cmake_minimum_required(VERSION 3.8)
project(cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(messages REQUIRED)
find_package(sensor_msgs REQUIRED)


add_executable(cpp_template_node
                src/template.cpp
                src/modules/Logger.cpp
)
ament_target_dependencies(
  cpp_template_node
  "rclcpp"
)

add_executable(cpp_subscriber_node
                src/cpp_subscriber.cpp
)
ament_target_dependencies(
  cpp_subscriber_node
  "rclcpp"
  "std_msgs"
  "std_srvs"
  "messages"
)

add_executable(cpp_publisher_node
                src/cpp_publisher.cpp
)
ament_target_dependencies(
  cpp_publisher_node
  "rclcpp"
  "messages"
)

add_executable(cpp_service_node
                src/cpp_service.cpp
)
ament_target_dependencies(
  cpp_service_node
  "rclcpp"
  "std_srvs"
)

install(TARGETS
  cpp_template_node
  cpp_subscriber_node
  cpp_publisher_node
  cpp_service_node
  DESTINATION lib/${PROJECT_NAME})
ament_package()