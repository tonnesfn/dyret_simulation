cmake_minimum_required(VERSION 3.1)
project(dyret_gazebo_plugin CXX)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
	dyret_common
	gazebo_dev
	roscpp
	std_srvs
	)


catkin_package(
	LIBRARIES ${PROJECT_NAME}
	DEPENDS gazebo
	)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	)

## Declare a C++ library
add_library(${PROJECT_NAME}
	src/controller.cpp
	)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
	${GAZEBO_LIBRARIES}
	)

install(TARGETS ${PROJECT_NAME}
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTIONATION})
