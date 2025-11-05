cmake_minimum_required(VERSION 3.8)
project(follower_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(follower_node src/follower_node.cpp)
ament_target_dependencies(follower_node
        rclcpp
        geometry_msgs
        nav_msgs
        gazebo_msgs
        sensor_msgs
        visualization_msgs
)
install(TARGETS follower_node DESTINATION lib/${PROJECT_NAME})


add_executable(leader_node src/leader_node.cpp)
ament_target_dependencies(leader_node
        rclcpp
        geometry_msgs
        gazebo_msgs
        sensor_msgs
        visualization_msgs
)
install(TARGETS leader_node DESTINATION lib/${PROJECT_NAME})



add_executable(measurement src/measurement.cpp)
ament_target_dependencies(measurement
        rclcpp
        geometry_msgs
        gazebo_msgs
        sensor_msgs
        visualization_msgs
        nav_msgs
)
install(TARGETS measurement DESTINATION lib/${PROJECT_NAME})



ament_package()

install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)