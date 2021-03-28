cmake_minimum_required(VERSION 3.0.2)
project(utility)
set(CMAKE_BUILD_TYPE "Release")


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    pcl_conversions
    pcl_ros
    gflags_catkin       # gflags
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)

# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})


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
    # /opt/ros/melodic/include
    # /usr/include/pcl-1.8
    /usr/include/eigen3
    ${catkin_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
)

add_executable(record_node src/record_node.cpp)
target_link_libraries(record_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# show a pointcloud.pcd 
add_executable(show_node src/show_pointcloud.cpp)
target_link_libraries(show_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable (process 
    src/process/main.cpp
    src/process/car_path.cpp
    src/process/scene_cloud.cpp
    src/process/process.cpp
)
target_link_libraries (process ${catkin_LIBRARIES} ${PCL_LIBRARIES})


# # extract model
# add_executable(extract_model_node 
#     src/process/extract_model.cpp
#     src/process/car_path.cpp
#     # src/process/scene_cloud.cpp
#     src/process/process.cpp
# )
# target_link_libraries(extract_model_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})


# development
add_executable(cylinder_node
    src/develop/cylinder.cpp
    src/develop/support_cylinder.cpp
    src/develop/support_base.cpp
)
target_link_libraries(cylinder_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})


# development
add_executable(pcd2ply_node 
    src/pcd2ply.cpp
)
target_link_libraries(pcd2ply_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(ply2pcd_node 
    src/ply2pcd.cpp
)
target_link_libraries(ply2pcd_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
