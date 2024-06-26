cmake_minimum_required(VERSION 3.0.2)
project(survive_publisher)

## 编译选项
add_compile_options(-std=c++14)

## 查找catkin依赖
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  geometry_msgs tf
  )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

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

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES survive_publisher
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/main.cpp)
add_executable(survive_ros_node src/survive_ros.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
#set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  survive
  )
target_link_libraries(survive_ros_node
  ${catkin_LIBRARIES}
  )

## Mark executables and/or libraries for installation
 install(TARGETS ${PROJECT_NAME}_node survive_ros_node
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
