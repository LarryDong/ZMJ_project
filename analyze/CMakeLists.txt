cmake_minimum_required(VERSION 3.0.2)
project(analyze)
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
find_package(PCL REQUIRED)
find_package(gflags REQUIRED)


catkin_package()

include_directories(
    include
    /usr/include/eigen3
    ${catkin_INCLUDE_DIRS}
)

add_executable(my_test src/test.cpp)
target_link_libraries(my_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} gflags)


add_executable(save_data src/save_data/save_data.cpp)
target_link_libraries(save_data ${catkin_LIBRARIES} ${PCL_LIBRARIES} gflags)
