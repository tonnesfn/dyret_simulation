cmake_minimum_required(VERSION 2.8.3)
project(dyret_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dyret_gazebo
#  CATKIN_DEPENDS gazebo_ros gazebo_ros_control roslaunch xacro
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Add launch files to be checked for proper formatting
roslaunch_add_file_check(launch)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############
install(DIRECTORY
	launch config
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
