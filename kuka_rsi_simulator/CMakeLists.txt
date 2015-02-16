cmake_minimum_required(VERSION 2.8.3)
project(kuka_rsi_simulator)

find_package(catkin REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kuka_sim
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

#############
## Install ##
#############

install(PROGRAMS kuka_rsi_simulator DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

#############
## Testing ##
#############

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
