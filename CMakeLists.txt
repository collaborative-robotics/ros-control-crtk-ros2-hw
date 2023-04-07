cmake_minimum_required (VERSION 3.8)
project (crtk_ros2_hw)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package (ament_cmake REQUIRED)
find_package (hardware_interface REQUIRED)
find_package (pluginlib REQUIRED)
find_package (rclcpp REQUIRED)
find_package (rclcpp_lifecycle REQUIRED)
find_package (sensor_msgs REQUIRED)
find_package (crtk_msgs REQUIRED)
find_package (rclcpp_action)
find_package (control_msgs)
find_package (geometry_msgs)
find_package (std_msgs)
find_package (trajectory_msgs)
find_package (tf2)


# library for hardware interface
add_library (${PROJECT_NAME} SHARED src/crtk_ros2_hardware_interface.cpp)

target_include_directories (crtk_ros2_hw PUBLIC
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include/${PROJECT_NAME}>
	)
  if(BUILD_TESTING)

    find_package(ament_lint_auto REQUIRED)
    list(APPEND AMENT_LINT_AUTO_EXCLUDE
      ament_cmake_uncrustify
      ament_cmake_cpplint
    )
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gmock REQUIRED)
    find_package(hardware_interface REQUIRED)

    ament_add_gmock(test_load_crtk_ros2_hw test/test_load_crtk_ros2_hw.cpp)
    ament_target_dependencies(test_load_crtk_ros2_hw
      pluginlib
      hardware_interface
      sensor_msgs
      crtk_msgs
    )
  endif()

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  control_msgs
  geometry_msgs
  sensor_msgs
  crtk_msgs
  trajectory_msgs)

pluginlib_export_plugin_description_file(hardware_interface crtk_ros2_hw.xml)



#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
	DIRECTORY include/
	DESTINATION include
)

# EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  control_msgs
  geometry_msgs
  sensor_msgs
  crtk_msgs
  trajectory_msgs
  )
ament_package ()
