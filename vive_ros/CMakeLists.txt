cmake_minimum_required(VERSION 3.0.2)
project(survive_publisher)

## 编译选项
add_compile_options(-std=c++14)

## 查找catkin依赖
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  geometry_msgs 
  tf
  arm_control
  message_generation
  message_runtime
  )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  joystick.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES survive_publisher
CATKIN_DEPENDS message_runtime message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
)
###########
## Build ##
###########

message( ${catkin_INCLUDE_DIRS} )
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  include()
  ../libsurvive/include
  ../libsurvive/redist  
  ../libsurvive/libs/cnkalman/libs/cnmatrix/include
)

link_directories(../libsurvive/build)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/main.cpp)
add_executable(survive_ros_node src/survive_ros.cpp)
add_executable(survive_publisher src/survive_publisher.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
#set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(survive_ros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(survive_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  survive
  )
target_link_libraries(survive_ros_node
  ${catkin_LIBRARIES}
  )
target_link_libraries(survive_publisher
  ${catkin_LIBRARIES}
  )

catkin_install_python(PROGRAMS
  script/joystick.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
 install(TARGETS ${PROJECT_NAME}_node survive_ros_node survive_publisher
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY ../libsurvive/include/libsurvive
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
