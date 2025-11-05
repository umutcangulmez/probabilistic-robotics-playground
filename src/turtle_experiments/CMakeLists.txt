cmake_minimum_required(VERSION 3.5)
project(turtle_experiments)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(turtle_experiments src/turtle_experiments.cpp)
ament_target_dependencies(turtle_experiments rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS
        turtle_experiments
        DESTINATION lib/${PROJECT_NAME}
)
# Install launch files
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)
ament_package()
