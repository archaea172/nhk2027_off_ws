cmake_minimum_required(VERSION 3.8)
project(control_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(ament_cmake REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include
  include/${PROJECT_NAME}
)

add_library(mppi_arm
  src/mppi_arm.cpp
)
target_link_libraries(mppi_arm
  Eigen3::Eigen
  kdl_parser::kdl_parser
)


add_executable(test_mppi
  src/test_mppi.cpp
)
ament_target_dependencies(test_mppi
  rclcpp
  std_msgs
)
target_link_libraries(test_mppi
  mppi_arm
)
install(
  TARGETS test_mppi
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
