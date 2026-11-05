cmake_minimum_required(VERSION 3.22)
project(cr5_moveit)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Threads REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

add_executable(cr5_read_pose src/cr5_read_pose.cpp)
target_compile_features(cr5_read_pose PUBLIC cxx_std_17)
ament_target_dependencies(cr5_read_pose rclcpp moveit_ros_planning_interface)
target_link_libraries(cr5_read_pose Threads::Threads)
install(TARGETS cr5_read_pose DESTINATION lib/${PROJECT_NAME})

add_executable(cr5_cartesian_line src/cr5_cartesian_line.cpp)
target_compile_features(cr5_cartesian_line PUBLIC cxx_std_17)
ament_target_dependencies(cr5_cartesian_line
  rclcpp geometry_msgs moveit_msgs moveit_core moveit_ros_planning_interface)
target_link_libraries(cr5_cartesian_line Threads::Threads)
install(TARGETS cr5_cartesian_line DESTINATION lib/${PROJECT_NAME})

add_executable(cr5_cartesian_circle src/cr5_cartesian_circle.cpp)
target_compile_features(cr5_cartesian_circle PUBLIC cxx_std_17)
ament_target_dependencies(cr5_cartesian_circle
  rclcpp geometry_msgs moveit_msgs moveit_core moveit_ros_planning_interface)
target_link_libraries(cr5_cartesian_circle Threads::Threads)
install(TARGETS cr5_cartesian_circle DESTINATION lib/${PROJECT_NAME})

add_executable(cr5_orbit src/cr5_orbit.cpp)
target_compile_features(cr5_orbit PUBLIC cxx_std_17)
ament_target_dependencies(cr5_orbit
  rclcpp geometry_msgs moveit_msgs moveit_core moveit_ros_planning_interface)
target_link_libraries(cr5_orbit Threads::Threads)
install(TARGETS cr5_orbit DESTINATION lib/${PROJECT_NAME})

add_executable(cr5_stepwise_orbit src/cr5_stepwise_orbit.cpp)
target_compile_features(cr5_stepwise_orbit PUBLIC cxx_std_17)
ament_target_dependencies(cr5_stepwise_orbit
  rclcpp geometry_msgs moveit_msgs moveit_core moveit_ros_planning_interface shape_msgs)
target_link_libraries(cr5_stepwise_orbit Threads::Threads)
install(TARGETS cr5_stepwise_orbit DESTINATION lib/${PROJECT_NAME})

add_executable(cr5_probe_sequence src/cr5_probe_sequence.cpp)
target_compile_features(cr5_probe_sequence PUBLIC cxx_std_17)
ament_target_dependencies(cr5_probe_sequence
  rclcpp geometry_msgs moveit_msgs moveit_core moveit_ros_planning_interface)
target_link_libraries(cr5_probe_sequence Threads::Threads)
install(TARGETS cr5_probe_sequence DESTINATION lib/${PROJECT_NAME})

add_executable(cr5_circular_scan src/cr5_circular_scan.cpp)
target_compile_features(cr5_circular_scan PUBLIC cxx_std_17)
ament_target_dependencies(cr5_circular_scan
  rclcpp geometry_msgs moveit_msgs moveit_core moveit_ros_planning_interface)
target_link_libraries(cr5_circular_scan Threads::Threads)
install(TARGETS cr5_circular_scan DESTINATION lib/${PROJECT_NAME})

add_executable(cr5_trace_end_effector src/cr5_trace_end_effector.cpp)
target_compile_features(cr5_trace_end_effector PUBLIC cxx_std_17)
ament_target_dependencies(cr5_trace_end_effector
  rclcpp geometry_msgs visualization_msgs moveit_ros_planning_interface)
target_link_libraries(cr5_trace_end_effector Threads::Threads)
install(TARGETS cr5_trace_end_effector DESTINATION lib/${PROJECT_NAME})

add_executable(cr5_scan_center_marker src/cr5_scan_center_marker.cpp)
target_compile_features(cr5_scan_center_marker PUBLIC cxx_std_17)
ament_target_dependencies(cr5_scan_center_marker
  rclcpp geometry_msgs visualization_msgs moveit_ros_planning_interface)
target_link_libraries(cr5_scan_center_marker Threads::Threads)
install(TARGETS cr5_scan_center_marker DESTINATION lib/${PROJECT_NAME})

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
