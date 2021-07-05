cmake_minimum_required(VERSION 3.0.2)
project(follow_color)

find_package(catkin REQUIRED COMPONENTS)

find_package(gazebo REQUIRED)

catkin_package()

include_directories(${GAZEBO_INCLUDE_DIRS})

add_library(targetplugin plugins/target.cc)
target_link_libraries(targetplugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS targetplugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY
  models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
	src/teleop_target.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)