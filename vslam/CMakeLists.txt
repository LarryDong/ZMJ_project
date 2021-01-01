cmake_minimum_required(VERSION 3.0.2)
project(vslam)
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
#  LIBRARIES slam
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

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

add_executable(vscanRegistration_node src/scanRegistration.cpp)
add_executable(vlaserOdometry_node src/laserOdometry.cpp)
add_executable(vlaserMapping_node src/laserMapping.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(vscanRegistration_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(vlaserOdometry_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(vlaserMapping_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
