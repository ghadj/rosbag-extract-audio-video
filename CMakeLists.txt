cmake_minimum_required(VERSION 2.8.3)
project(rosbag-extract-audio-video)

find_package(
  catkin REQUIRED
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
)

install(DIRECTORY scripts samples
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
