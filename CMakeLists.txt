cmake_minimum_required(VERSION 2.8.3)
project(safe_vel)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
  
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	sensor_msgs
	tf
)

include_directories( include ${catkin_INCLUDE_DIRS} )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES safe_vel
  CATKIN_DEPENDS roscpp
)

add_library(safe_vel src/safe_vel.cpp)
target_link_libraries(safe_vel ${catkin_LIBRARIES})
add_dependencies(safe_vel ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(safe_vel_node src/safe_vel_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(safe_vel_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(safe_vel_node safe_vel)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS safe_vel
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/safe_vel/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
)

install(TARGETS safe_vel_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
    

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_safe_vel.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
