cmake_minimum_required(VERSION 3.16)
project(joint_mode_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  angles
  control_msgs
  control_toolbox
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  rsl
  tl_expected
  trajectory_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(joint_mode_controller_parameters
  src/joint_mode_controller_parameters.yaml
)

add_library(joint_mode_controller SHARED
  src/joint_mode_controller.cpp
)
target_compile_features(joint_mode_controller PUBLIC cxx_std_17)
target_include_directories(joint_mode_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/joint_mode_controller>
)
target_link_libraries(joint_mode_controller PUBLIC
  joint_mode_controller_parameters
)
ament_target_dependencies(joint_mode_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(joint_mode_controller PRIVATE "JOINT_MODE_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface joint_mode_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/joint_mode_controller
)
install(TARGETS
    joint_mode_controller
    joint_mode_controller_parameters
  EXPORT export_joint_mode_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_joint_mode_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
