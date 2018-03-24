cmake_minimum_required(VERSION 2.8.3)
project(final)

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
)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages( DEPENDENCIES
	actionlib_msgs
	sensor_msgs
	geometry_msgs
	std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

catkin_package(CATKIN_DEPENDS 
	roscpp 
	sensor_msgs 
	actionlib_msgs
	roslib
)

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(Name src/Name.cpp)
#target_link_libraries(Name ${catkin_LIBRARIES})
#add_dependencies(Name ${catkin_EXPORTED_TARGETS})






















