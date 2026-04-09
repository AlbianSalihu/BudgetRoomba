cmake_minimum_required(VERSION 3.8)
project(br_state_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)





add_executable(odometry_generator src/odometry_generator.cpp)

add_executable(arena_to_map_estimator src/arena_to_map_estimator.cpp)


ament_target_dependencies(
    odometry_generator
    nav_msgs
    geometry_msgs
    rclcpp
    tf2_ros
)

ament_target_dependencies(
    arena_to_map_estimator
    nav_msgs
    geometry_msgs
    rclcpp
    tf2_ros
    grid_map_cv
    grid_map_ros
    OpenCV
    tf2_eigen
    cv_bridge
    sensor_msgs
)

install(TARGETS
  arena_to_map_estimator
  DESTINATION lib/${PROJECT_NAME}/
)

install(TARGETS
  odometry_generator
  DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)