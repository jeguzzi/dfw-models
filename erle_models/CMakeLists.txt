cmake_minimum_required(VERSION 2.8.3)
project(erle_models)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS)
include_directories(include ${catkin_INCLUDE_DIRS})

###################################
## catkin specific configuration ##
###################################

catkin_package(
  DEPENDS
)
