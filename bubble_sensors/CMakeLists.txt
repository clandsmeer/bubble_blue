cmake_minimum_required(VERSION 3.8)
project(bubble_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(libmavconn REQUIRED)
find_package(mavlink REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(dvl_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Add the executable and target
add_executable(generic_sensor_publisher src/generic_sensor.cpp)
add_executable(simulated_dvl src/simulated_dvl.cpp)
add_executable(sim_imu_message_converter src/sim_imu_message_converter.cpp)
add_executable(gravity_compensator src/gravity_compensator.cpp)
add_executable(dvl_message_converter src/dvl_message_converter.cpp)
add_executable(synchronizer src/synchronizer.cpp)
ament_target_dependencies(generic_sensor_publisher rclcpp nav_msgs)
ament_target_dependencies(simulated_dvl rclcpp nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)
ament_target_dependencies(sim_imu_message_converter rclcpp nav_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs)
ament_target_dependencies(gravity_compensator rclcpp nav_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs)
ament_target_dependencies(dvl_message_converter rclcpp geometry_msgs dvl_msgs)
ament_target_dependencies(synchronizer rclcpp geometry_msgs sensor_msgs)

add_library(mavros_sensor_plugin SHARED src/mavros_sensor_plugin.cpp)
target_include_directories(mavros_sensor_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(mavros_sensor_plugin
  mavros
  mavros_msgs
  libmavconn
  mavlink
  diagnostic_updater
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pluginlib)

# Tells mavros where to look for mavros plugins belonging to bubble_sensors
pluginlib_export_plugin_description_file(mavros mavros_plugins.xml)

install(
  TARGETS generic_sensor_publisher
  simulated_dvl
  sim_imu_message_converter
  gravity_compensator
  mavros_sensor_plugin
  dvl_message_converter
  synchronizer
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

install(FILES mavros_plugins.xml
  DESTINATION share/${PROJECT_NAME})

#install the dvl enabler
install(PROGRAMS
  src/dvl_enabler.py
  src/initializer.py
  src/vectornav_manager.py
  src/sim_vectornav_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

# add the custom service for IMU Configuration
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ConfigureVN100.srv"
  DEPENDENCIES std_msgs
)
# make sure the launch directory can be accessed so the overall bringup file can launch sensors
install(
  DIRECTORY launch config
  DESTINATION share/bubble_sensors
)

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
