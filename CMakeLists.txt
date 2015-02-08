cmake_minimum_required(VERSION 2.8.3)
project(pose_to_tf_publisher)



#######################################################################################################################
## packages
#######################################################################################################################


set(${PROJECT_NAME}_CATKIN_COMPONENTS
    cmake_modules
    roscpp
    nav_msgs
    std_msgs
    geometry_msgs
    tf2_msgs
    tf2
    tf2_ros
    rosconsole
    angles
    laserscan_to_pointcloud
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_COMPONENTS})
find_package(Boost REQUIRED)



catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
        pose_to_tf_publisher
    DEPENDS
        Boost
)



#######################################################################################################################
## build
#######################################################################################################################

#==================
# includes
#==================

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)



#==================
# libraries
#==================

add_library(pose_to_tf_publisher
    src/pose_to_tf_publisher.cpp
)


#==================
# executables
#==================

add_executable(pose_to_tf_publisher_node
    src/pose_to_tf_publisher_node.cpp
)



#==================
# library link
#==================

target_link_libraries(pose_to_tf_publisher
    ${catkin_LIBRARIES}
)

target_link_libraries(pose_to_tf_publisher_node
    pose_to_tf_publisher
    ${catkin_LIBRARIES}
)

