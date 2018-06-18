cmake_minimum_required(VERSION 3.1)
project(dyret_gazebo_plugin CXX)

# This is allowed as of Melodic
set(CMAKE_CXX_STANDARD 14)
# Set warning flags to get prettiest and good C++
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Werror -Wfloat-equal -Wno-unused -Wold-style-cast")

find_package(catkin REQUIRED COMPONENTS
	dyret_common
	gazebo_dev
	geometry_msgs
	roscpp
	std_srvs
	)


catkin_package(
	LIBRARIES ${PROJECT_NAME}
	DEPENDS gazebo
	)

# Include our code separate
include_directories(include)

# Include CATKIN packages as SYSTEM include so that warnings are not activated
include_directories(
	SYSTEM
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
