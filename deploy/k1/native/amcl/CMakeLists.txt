cmake_minimum_required(VERSION 3.16)
project(rvclaw_amcl_core_repair LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
set(dependencies rclcpp rclcpp_lifecycle rclcpp_components message_filters
  tf2_geometry_msgs geometry_msgs nav_msgs sensor_msgs std_srvs tf2_ros tf2
  nav2_util nav2_msgs pluginlib)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()
add_library(amcl_core SHARED amcl_node.cpp)
target_compile_features(amcl_core PUBLIC cxx_std_17)
target_compile_definitions(amcl_core PRIVATE HAVE_DRAND48)
target_include_directories(amcl_core PRIVATE /opt/ros/humble/include)
ament_target_dependencies(amcl_core ${dependencies})
target_link_libraries(amcl_core /opt/ros/humble/lib/libmap_lib.so
  /opt/ros/humble/lib/libpf_lib.so /opt/ros/humble/lib/libsensors_lib.so)
install(TARGETS amcl_core LIBRARY DESTINATION lib)
