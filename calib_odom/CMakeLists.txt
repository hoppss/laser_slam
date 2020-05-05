cmake_minimum_required(VERSION 2.8.3)
project(calib_odom)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_conversions
  pcl_ros
)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  open_karto
  calib_odom
  CATKIN_DEPENDS roscpp std_msgs
 # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIR}
  ${DLib_INCLUDE_DIRS}
  ${DBoW2_INCLUDE_DIRS}
)

link_directories(/usr/local/lib)

add_executable(calib_odom_node src/Odom_Calib.cpp
src/main.cpp)
## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(calib_odom_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(calib_odom_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${CERES_LIBRARIES}
    libcsm.so
 )