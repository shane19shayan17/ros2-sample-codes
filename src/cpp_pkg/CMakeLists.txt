cmake_minimum_required(VERSION 3.8)
project(cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
add_executable(cpp_node
                src/cpp_node.cpp
                src/modules/Run.cpp
)
ament_target_dependencies(
  cpp_node
  "rclcpp"
)
target_include_directories(cpp_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(cpp_node PUBLIC c_std_99 cxx_std_17)
add_executable(cpp_subscriber
                src/cpp_subscriber.cpp
)
ament_target_dependencies(
  cpp_subscriber
  "rclcpp"
  "example_interfaces"
)
add_executable(cpp_publisher
                src/cpp_publisher.cpp
)
ament_target_dependencies(
  cpp_publisher
  "rclcpp"
  "example_interfaces"
)
install(TARGETS
  cpp_node
  cpp_subscriber
  cpp_publisher
  DESTINATION lib/${PROJECT_NAME})
ament_package()