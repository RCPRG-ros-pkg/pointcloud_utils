cmake_minimum_required(VERSION 2.8)
project(pointcloud_utils)

find_package(catkin REQUIRED COMPONENTS
    rtt_ros rtt_rosclock std_msgs interactive_markers tf pcl_ros pcl_conversions eigen_conversions cmake_modules collision_convex_model
 octomap_msgs kdl_conversions pointcloud_utils_msgs planer_utils
  )

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

link_directories(${orocos_kdl_LIBRARY_DIRS})

include_directories(${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${USE_OROCOS_INCLUDE_DIRS})

add_executable(find_features_pc src/find_features_pc.cpp)
add_executable(pc_filter src/pc_filter.cpp)

target_link_libraries(find_features_pc ${catkin_LIBRARIES})
target_link_libraries(pc_filter ${catkin_LIBRARIES})

catkin_package(
    CATKIN_DEPENDS collision_convex_model
)

