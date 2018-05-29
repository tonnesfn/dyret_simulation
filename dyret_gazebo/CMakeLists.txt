cmake_minimum_required(VERSION 3.1)
project(dyret_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
)

catkin_package()

roslaunch_add_file_check(launch)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
