cmake_minimum_required(VERSION 2.8.3)
project(imu_uwb_fusion)

add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  common_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)

 add_message_files(
   FILES
   UwbMsg.msg

   LinktrackAnchorframe0.msg
   LinktrackNode0.msg
   LinktrackNode1.msg
   LinktrackNode2.msg
   LinktrackNodeframe0.msg
   LinktrackNodeframe1.msg
   LinktrackNodeframe2.msg
   LinktrackNodeframe3.msg
   LinktrackTag.msg
   LinktrackTagframe0.msg
   TofsenseCascade.msg
   TofsenseFrame0.msg
   LinktrackAoaNode0.msg
   LinktrackAoaNodeframe0.msg
   LinktrackNode4Anchor.msg
   LinktrackNode4Tag.msg
   LinktrackNodeframe4.msg
   LinktrackNode5.msg
   LinktrackNodeframe5.msg
   LinktrackNode6.msg
   LinktrackNodeframe6.msg
   TofsenseMFrame0.msg
   TofsenseMFrame0Pixel.msg
   IotFrame0.msg
   IotFrame0Node.msg
 )

  generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES imu_uwb_fusion
  CATKIN_DEPENDS message_runtime roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(imu_uwb_fusion src/imu_uwb_fusion.cpp)

add_executable(imu_uwb_fusion_node src/imu_uwb_fusion_node.cpp)
target_link_libraries(imu_uwb_fusion_node ${catkin_LIBRARIES} imu_uwb_fusion)

# add_executable(moving_average_filter_node src/moving_average_filter.cpp)
# target_link_libraries(moving_average_filter_node ${catkin_LIBRARIES})

#add_executable(uwb_plot_node src/uwb_plot.cpp)
#target_link_libraries(uwb_plot_node ${catkin_LIBRARIES})
