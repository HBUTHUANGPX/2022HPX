cmake_minimum_required(VERSION 3.1.3)
project(arm_moveit_2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_msgs
  control_msgs
  moveit_ros_planning_interface
  actionlib
  actionlib_msgs
  sensor_msgs
)
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   actionlib_msgs
# )

catkin_package(

#  INCLUDE_DIRS include
#  LIBRARIES demo04_action

 CATKIN_DEPENDS roscpp rospy std_msgs actionlib actionlib_msgs moveit_msgs control_msgs sensor_msgs

#  DEPENDS system_lib

)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(my_action_server src/my_action_server.cpp)
add_dependencies(my_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(my_action_server
  ${catkin_LIBRARIES}
)
add_executable(ps4_control src/ps4_control.cpp)
target_link_libraries(ps4_control
  ${catkin_LIBRARIES}
)
add_executable(fake_joint_fresh src/fake_joint_fresh.cpp)
target_link_libraries(fake_joint_fresh
  ${catkin_LIBRARIES}
)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
