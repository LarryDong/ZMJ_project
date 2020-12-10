cmake_minimum_required(VERSION 3.0.2)
project(reconstruct)
set(CMAKE_BUILD_TYPE "Release")


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    pcl_conversions
    pcl_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mapping
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    /usr/local/include/opencv4
    # /opt/ros/melodic/include
    # /usr/include/pcl-1.8
    /usr/include/eigen3
    ${catkin_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
)


add_executable(mapping_node src/mapping.cpp)
target_link_libraries(mapping_node  ${catkin_LIBRARIES})

add_executable(my_test src/test.cpp)
target_link_libraries(my_test)