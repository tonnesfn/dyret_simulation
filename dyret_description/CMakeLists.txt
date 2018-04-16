cmake_minimum_required(VERSION 2.8.3)
project(dyret_description)

find_package(catkin REQUIRED COMPONENTS
	roslaunch
	)

catkin_package()

## Add launch files to be checked for proper formatting
roslaunch_add_file_check(launch)

install(DIRECTORY
	launch meshes urdf
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	)
