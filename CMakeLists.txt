cmake_minimum_required(VERSION 3.0.2)
project(low_pass_force_torque_sensor_controller)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  geometry_msgs
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
  dynamic_reconfigure
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/LowPassFilter.cfg
)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
    controller_interface
    geometry_msgs
    hardware_interface
    realtime_tools
    roscpp
    dynamic_reconfigure
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)


###########
## Build ##
###########

# Specify header include paths
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/low_pass_force_torque_sensor_controller.cpp
  src/low_pass_filter.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install targets
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Intall plugins
install(FILES low_pass_force_torque_sensor_plugin.xml
              config/low_pass_force_torque_sensor_controller.yaml
              launch/low_pass_force_torque_sensor_controller.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
