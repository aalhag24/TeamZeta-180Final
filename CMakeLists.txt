cmake_minimum_required(VERSION 2.8.3)
project(Final)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	tf2 
	tf2_ros 
	actionlib
	actionlib_msgs
	sensor_msgs
	geometry_msgs
	std_msgs
	amcl
	roslib
	roslaunch

	base_local_planner
	cmake_modules
	costmap_2d
	dynamic_reconfigure
	move_base
	move_base_msgs
	nav_core
	nav_msgs
	pluginlib

	rotate_recovery
	base_local_planner
	carrot_planner
	clear_costmap_recovery
	logical_camera_plugin
	laser_geometry
	
	message_generation
)

find_package(cmake_modules REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  LocalScan.msg
  Num.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages( DEPENDENCIES
	std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

catkin_package(INCLUDE_DIRS include
	LIBRARIES pose_follower
	CATKIN_DEPENDS

	roscpp 
	sensor_msgs 
	actionlib_msgs
	roslib

	base_local_planner 
	cmake_modules 
	costmap_2d 
	dynamic_reconfigure 
	move_base 
	move_base_msgs 	
	actionlib
	nav_core 
	nav_msgs 
	pluginlib
	Costmap2DPublisher

	my_tf_configuration_dep 
	my_odom_configuration_dep 
	my_sensor_configuration_dep

	rotate_recovery
	base_local_planner
	carrot_planner
	clear_costmap_recovery
	logical_camera_plugin
	laser_geometry

	message_runtime
)

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS})
#add_dependencies(${PROJECT_NAME} node ${${PROJECT_NAME}_EXPORTED_TARGETS} )

add_executable(logicalCamera src/logicalCamera.cpp)
target_link_libraries(logicalCamera ${catkin_LIBRARIES})
add_dependencies(logicalCamera ${catkin_EXPORTED_TARGETS})

add_executable(Map src/Map.cpp)
target_link_libraries(Map ${catkin_LIBRARIES})
add_dependencies(Map ${catkin_EXPORTED_TARGETS})

add_executable(saferandomwalk src/saferandomwalk.cpp)
target_link_libraries(saferandomwalk ${catkin_LIBRARIES})
add_dependencies(saferandomwalk ${catkin_EXPORTED_TARGETS})
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation






