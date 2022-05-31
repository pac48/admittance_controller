cmake_minimum_required(VERSION 3.5)
project(admittance_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(control_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(controller_interface REQUIRED)
find_package(ik_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(joint_limits_interface REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(rcutils REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(RL REQUIRED)


# The admittance controller
add_library(admittance_controller SHARED
        src/admittance_controller.cpp
)
#add_library(my_admittance_controller SHARED
#        src/admittance_controller.cpp
#        )
add_library(moveit_differential_ik_plugin SHARED # this is broken currently
        src/moveit_kinematics.cpp
        )
add_library(rl_differential_ik_plugin SHARED
        src/rl_kinematics.cpp
        )

target_include_directories(
  admittance_controller
  PRIVATE
  include
)
#target_include_directories(
#        my_admittance_controller
#        PRIVATE
#        include
#)
target_include_directories(
        moveit_differential_ik_plugin
        PRIVATE
        include
)
target_include_directories(
       rl_differential_ik_plugin
        PRIVATE
        include
)

target_link_libraries(
  admittance_controller
)
#target_link_libraries(
#        my_admittance_controller
#)
target_link_libraries(
        moveit_differential_ik_plugin
)
target_link_libraries(
        rl_differential_ik_plugin
)


ament_target_dependencies(
  admittance_controller
  backward_ros
  control_msgs
  control_toolbox
  controller_interface
        ik_interface
  ${Eigen_LIBRARIES}
  filters
  geometry_msgs
  hardware_interface
  joint_limits_interface
  joint_trajectory_controller
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
        angles
)
#ament_target_dependencies(
#        my_admittance_controller
#        backward_ros
#        control_msgs
#        control_toolbox
#        controller_interface
#        ik_interface
#        ${Eigen_LIBRARIES}
#        filters
#        geometry_msgs
#        hardware_interface
#        joint_limits_interface
#        joint_trajectory_controller
#        pluginlib
#        rclcpp
#        rclcpp_lifecycle
#        realtime_tools
#        tf2
#        tf2_eigen
#        tf2_geometry_msgs
#        tf2_ros
#        angles
#)
ament_target_dependencies(
        moveit_differential_ik_plugin
        rcutils
        geometry_msgs
        trajectory_msgs
        ik_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        realtime_tools
        tf2
        tf2_eigen
        tf2_geometry_msgs
        tf2_ros
        angles
        moveit_ros_planning
)
ament_target_dependencies(
        rl_differential_ik_plugin
        rcutils
        geometry_msgs
        trajectory_msgs
        ik_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
        realtime_tools
        tf2
        tf2_eigen
        tf2_geometry_msgs
        tf2_ros
        angles
        RL
)


# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(admittance_controller PRIVATE "ADMITTANCE_CONTROLLER_BUILDING_DLL")
#target_compile_definitions(my_admittance_controller PRIVATE "MY_ADMITTANCE_CONTROLLER_BUILDING_DLL")
target_compile_definitions(moveit_differential_ik_plugin PRIVATE "IK_BUILDING_DLL")
target_compile_definitions(rl_differential_ik_plugin PRIVATE "RL_IK_BUILDING_DLL")


pluginlib_export_plugin_description_file(controller_interface admittance_controller.xml)
pluginlib_export_plugin_description_file(ik_interface moveit_kinematics.xml)
pluginlib_export_plugin_description_file(ik_interface rl_kinematics.xml)


install(
  TARGETS admittance_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
#install(
#        TARGETS my_admittance_controller
#        RUNTIME DESTINATION bin
#        ARCHIVE DESTINATION lib
#        LIBRARY DESTINATION lib
#)
install(
        TARGETS moveit_differential_ik_plugin
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)
install(
        TARGETS rl_differential_ik_plugin
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

#set(BUILD_TESTING 0)
#if(BUILD_TESTING)
#  find_package(ament_cmake_gmock REQUIRED)
#  find_package(controller_manager REQUIRED)
#  find_package(hardware_interface REQUIRED)
#  find_package(ros2_control_test_assets REQUIRED)
#
#  ament_add_gmock(test_load_admittance_controller test/test_load_admittance_controller.cpp)
#  target_include_directories(test_load_admittance_controller PRIVATE include)
#  ament_target_dependencies(
#    test_load_admittance_controller
#    controller_manager
#    hardware_interface
#    ros2_control_test_assets
#  )
#
#  ament_add_gmock(test_admittance_controller test/test_admittance_controller.cpp)
#  target_include_directories(test_admittance_controller PRIVATE include)
#  target_link_libraries(test_admittance_controller admittance_controller)
#  ament_target_dependencies(
#    test_admittance_controller
#    control_msgs
#    controller_interface
#    hardware_interface
#    ros2_control_test_assets
#  )
#endif()

ament_export_include_directories(
  include
)

ament_export_libraries(
  admittance_controller
)
#ament_export_libraries(
#        my_admittance_controller
#)
ament_export_libraries(
        moveit_differential_ik_plugin
)
ament_export_libraries(
        rl_differential_ik_plugin
)

ament_export_dependencies(
  backward_ros
  control_msgs
  control_toolbox
  controller_interface
        ik_interface
  filters
  geometry_msgs
  hardware_interface
  joint_limits_interface
  joint_trajectory_controller
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)

ament_package()
