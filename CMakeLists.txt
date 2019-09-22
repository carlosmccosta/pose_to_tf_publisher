cmake_minimum_required(VERSION 2.8.3)
project(pose_to_tf_publisher)


##############
## packages ##
##############


set(${PROJECT_NAME}_CATKIN_COMPONENTS
    angles
    geometry_msgs
    laserscan_to_pointcloud
    nav_msgs
    rosconsole
    roscpp
    std_msgs
    tf2
    tf2_msgs
    tf2_ros
)


find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_COMPONENTS})



###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        pose_to_tf_publisher
    CATKIN_DEPENDS
        ${${PROJECT_NAME}_CATKIN_COMPONENTS}
    DEPENDS
        Boost
        EIGEN3
)




###########
## Build ##
###########

#===========
# includes =
#===========

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)



#============
# libraries =
#============

add_library(pose_to_tf_publisher
    src/pose_to_tf_publisher.cpp
)


#==============
# executables =
#==============

add_executable(pose_to_tf_publisher_node
    src/pose_to_tf_publisher_node.cpp
)


#===============
# dependencies =
#===============

add_dependencies(pose_to_tf_publisher
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(pose_to_tf_publisher_node
    pose_to_tf_publisher
    ${catkin_EXPORTED_TARGETS}
)


#=================
# libraries link =
#=================

target_link_libraries(pose_to_tf_publisher
    ${catkin_LIBRARIES}
)


#===================
# executables link =
#===================

target_link_libraries(pose_to_tf_publisher_node
    pose_to_tf_publisher
    ${catkin_LIBRARIES}
)



#############
## Install ##
#############

#============================
# Libraries and executables =
#============================

install(
    TARGETS
        pose_to_tf_publisher
        pose_to_tf_publisher_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


#================
# Header files =
#================

install(
    DIRECTORY
        include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


#===============
# Launch files =
#===============

install(
    DIRECTORY
        launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
