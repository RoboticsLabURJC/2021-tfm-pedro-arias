cmake_minimum_required(VERSION 3.0.2)
project(follow_person)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY
  models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
