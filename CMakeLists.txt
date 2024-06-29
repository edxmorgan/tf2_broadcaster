cmake_minimum_required(VERSION 3.16)
project(tf2_broadcaster LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=conversion -Werror=unused-but-set-variable
                      -Werror=return-type -Werror=shadow -Werror=format)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  tf2_msgs
  tf2
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
  tf2_broadcaster_parameters
  src/tf2_broadcaster_parameters.yaml
)

add_library(tf2_broadcaster SHARED
  src/tf2_broadcaster_base.cpp
  src/tf2_broadcaster.cpp
)

target_link_libraries(tf2_broadcaster PUBLIC casadi)

target_compile_features(tf2_broadcaster PUBLIC cxx_std_17)
target_include_directories(tf2_broadcaster PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/tf2_broadcaster>
)
target_link_libraries(tf2_broadcaster PUBLIC
  tf2_broadcaster_parameters
)
ament_target_dependencies(tf2_broadcaster PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(tf2_broadcaster PRIVATE "TF2_BROADCASTER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface tf2_broadcaster_plugin.xml)


if(BUILD_TESTING)


endif()

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS
    ${PROJECT_NAME}
    tf2_broadcaster_parameters
  EXPORT export_tf2_broadcaster
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_tf2_broadcaster HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
