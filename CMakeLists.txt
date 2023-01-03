cmake_minimum_required (VERSION 3.8)
project (ros-control-crtk-ros2-hw)

# find dependencies
find_package (ament_cmake REQUIRED)
find_package (hardware_interface REQUIRED)
find_package (pluginlib REQUIRED)
find_package (rclcpp REQUIRED)
find_package (rclcpp_lifecycle REQUIRED)
find_package (sensor_msgs REQUIRED)
find_package (controller_manager)
find_package (rclcpp_action)
find_package (control_msgs)
find_package (geometry_msgs)
find_package (std_msgs)
find_package (trajectory_msgs)
find_package (tf2)

# library for hardware interface
add_library (ros_control_crtk_ros_hw src/crtk_ros_hardware_interface.cpp)
target_include_directories (ros_control_crtk_ros_hw PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>)

ament_target_dependencies (ros_control_crtk_ros_hw
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  controller_manager
  rclcpp_action
  control_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs)

pluginlib_export_plugin_description_file(hardware_interface ros-control-crtk-ros-hw.xml)

#############
## Install ##
#############

# install (DIRECTORY launch/ DESTINATION ${AMENT_PACKAGE_SHARE_DESTINATION}/launch)
# install (DIRECTORY config/ DESTINATION ${AMENT_PACKAGE_SHARE_DESTINATION}/config)

## Mark executables and/or libraries for installation
install (TARGETS ros_control_crtk_ros_hw #--------------------------- add executable here
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

## Mark cpp header files for installation
install (DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h")

ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package ()
