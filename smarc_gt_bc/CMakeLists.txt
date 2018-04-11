cmake_minimum_required(VERSION 2.8.3)
project(smarc_gt_bc)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" HAS_FLAGS)
if(HAS_FLAGS)
    add_compile_options(-std=c++11 -Wall -Wextra)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES odom_tf_bc
  CATKIN_DEPENDS roscpp tf rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

 add_executable(ground_truth_bc_node
     src/ground_truth_bc_node.cpp
     )

 add_dependencies(ground_truth_bc_node
     ${${PROJECT_NAME}_EXPORTED_TARGETS}
     ${catkin_EXPORTED_TARGETS}
     )

## Specify libraries to link a library or executable target against
 target_link_libraries(ground_truth_bc_node
   ${catkin_LIBRARIES}
 )


