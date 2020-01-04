cmake_minimum_required(VERSION 2.8.3)
project(dji_m100_controllers_gazebo)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  visualization_msgs
  gazebo_ros
  gazebo_dev
  gazebo_ros_control
  hector_quadrotor_interface
  hector_quadrotor_controllers
)
include_directories(include ${catkin_INCLUDE_DIRS} /usr/include/gazebo-9/)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dji_m100_controller_gazebo_sitl
  CATKIN_DEPENDS roscpp
  std_msgs
  sensor_msgs
  visualization_msgs
  gazebo_ros
  gazebo_ros_control hector_quadrotor_interface hector_quadrotor_controllers
)

### Create HITL Library.
add_library(dji_m100_controller_gazebo_hitl SHARED src/dji_m100_ros_hitl_controller_gazebo.cpp)
target_link_libraries(dji_m100_controller_gazebo_hitl ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
install(TARGETS dji_m100_controller_gazebo_hitl
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


### Create SITL Library
add_library(dji_m100_controller_gazebo_sitl src/dji_m100_ros_sitl_controller_gazebo.cpp)
target_link_libraries(dji_m100_controller_gazebo_sitl ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS dji_m100_controller_gazebo_sitl
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

install(FILES
  dji_m100_controller_gazebo.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
