cmake_minimum_required(VERSION 2.8.3)
project(behavior_tree_leaves)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  genmsg
  actionlib_msgs
  actionlib
  message_generation
  roscpp
  roslib
  rospy
  std_msgs
  behavior_tree_core
  roslaunch
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

roslaunch_add_file_check(launch)
add_definitions(-Wall -g -O0 -Wno-deprecated -static -Bstatic -std=gnu++0x)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
  DEPENDS Eigen
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include
)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${kinova_driver_INCLUDE_DIRS})
## Declare a cpp library
# add_library(learning_actionlib
#   src/${PROJECT_NAME}/learning_actionlib.cpp
# )

link_directories(/home/lni/sim_ws/devel/lib)
link_directories(/usr/lib/python2.7/config-x86_64-linux-gnu)
set(COMMON_LIB ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_executable(action_client src/action_client.cpp)
target_link_libraries(
  action_client
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)
add_dependencies(
 action_client
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)


add_executable(condition_client src/condition_client.cpp)
target_link_libraries(
  condition_client
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)
add_dependencies(
 condition_client
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp

)

add_executable(action_example example_nodes/cpp/action_example.cpp)
target_link_libraries(
  action_example
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)
add_dependencies(
 action_example
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)


add_executable(condition_example example_nodes/cpp/condition_example.cpp)
target_link_libraries(
  condition_example
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)
add_dependencies(
 condition_example
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)


add_executable(pick_obj nodes/cpp/pick_obj.cpp)
target_link_libraries(pick_obj python2.7 ${catkin_LIBRARIES}  ${kinova_driver_LIBRARIES})
add_dependencies(
 pick_obj
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(if_release nodes/cpp/if_release.cpp)
target_link_libraries(if_release python2.7 ${catkin_LIBRARIES}  ${kinova_driver_LIBRARIES})
add_dependencies(
 if_release
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(open_gripper nodes/cpp/open_gripper.cpp)
target_link_libraries(open_gripper python2.7 ${catkin_LIBRARIES}  ${kinova_driver_LIBRARIES})
add_dependencies(
 open_gripper
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(close_gripper nodes/cpp/close_gripper.cpp)
target_link_libraries(close_gripper  python2.7 ${catkin_LIBRARIES}  ${kinova_driver_LIBRARIES})
add_dependencies(
 close_gripper
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(place_obj nodes/cpp/place_obj.cpp)
target_link_libraries(place_obj python2.7 ${catkin_LIBRARIES}  ${kinova_driver_LIBRARIES})
add_dependencies(
 place_obj
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

add_executable(go_home nodes/cpp/go_home.cpp)
target_link_libraries(go_home python2.7 ${catkin_LIBRARIES}  ${kinova_driver_LIBRARIES})
add_dependencies(
 go_home
  ${behavior_tree_leaves_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  behavior_tree_core_generate_messages_cpp
)

#add_executable(test nodes/cpp/test.cpp)
#target_link_libraries(test  python2.7 ${catkin_LIBRARIES}  ${kinova_driver_LIBRARIES})

