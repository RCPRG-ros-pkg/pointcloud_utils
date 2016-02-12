cmake_minimum_required(VERSION 2.8)
project(pointcloud_filter)

find_package(catkin REQUIRED COMPONENTS
    rtt_ros rtt_rosclock std_msgs interactive_markers tf pcl_ros pcl_conversions eigen_conversions cmake_modules collision_convex_model
 octomap_msgs barrett_hand_controller_msgs kdl_conversions
  )

find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

link_directories(${orocos_kdl_LIBRARY_DIRS})

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${USE_OROCOS_INCLUDE_DIRS})

add_executable(pc_filter src/pc_filter.cpp)

target_link_libraries(pc_filter ${catkin_LIBRARIES})

catkin_package(
    DEPENDS ompl eigen
    CATKIN_DEPENDS collision_convex_model
)

