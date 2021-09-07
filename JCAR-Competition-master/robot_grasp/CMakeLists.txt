cmake_minimum_required(VERSION 2.8.3)
project(robot_grasp)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
#s et(PCL_lib_DIR /opt/PCL/pcl/release/lib)


#find_package(Eigen3 REQUIRED)
#set(CMAKE_PREFIX_PATH "/opt/PCL/pcl/release")
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV)
find_package(catkin REQUIRED COMPONENTS
        tf
	dynamic_reconfigure
	message_generation
	pcl_conversions
        pcl_ros
	roscpp
	rospy
	genmsg
	roslint
	std_msgs
	sensor_msgs
        cv_bridge
        moveit_core
        moveit_ros_planning
        moveit_ros_planning_interface
        
)

catkin_python_setup()
add_message_files(
	FILES
	DetectedObject.msg
	DetectedObjectsArray.msg
        Detected_obj_param.msg
        Detected_obj_param_Array.msg
        VFH.msg
        tramsform_obj_param.msg
        real_detected_obj.msg
        real_detected_obj_array.msg
)
find_package(kinova_driver REQUIRED)
add_service_files(
   FILES
   GetFloatArrayFeature.srv
   GetNormals.srv
   GetVfh.srv
   SprawnModel.srv
   Centroid_Compute.srv
   coordinate_transform.srv
   VFH_pick_place.srv
   G_pick_place.srv
   calibration_transform.srv
   real_kinova_pick.srv
)

generate_messages(
	DEPENDENCIES
        std_msgs
	sensor_msgs
)

# generate_dynamic_reconfigure_options(
#   config/Pcl.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
	 INCLUDE_DIRS include
   LIBRARIES robot_grasp
   CATKIN_DEPENDS message_runtime
   DEPENDS Eigen
   pcl_conversions pcl_ros
#  DEPENDS system_lib
)

# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_include_DIR}
  ${Eigen_INCLUDE_DIRS}
)

link_directories(
  lib
  ${PCL_INCLUDE_DIRS}
)
include_directories(${kinova_driver_INCLUDE_DIRS})


#add_executable(pick_place_ src/pick_place_.cpp)
#target_link_libraries(pick_place_ ${catkin_LIBRARIES} libboost_thread.so)

add_executable(transform src/transform.cpp)
target_link_libraries(transform ${catkin_LIBRARIES})

add_executable(centroid_compute src/centroid_compute.cpp)
target_link_libraries(centroid_compute ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)

add_executable(coordinates_transformer src/coordinates_transformer.cpp)
target_link_libraries(coordinates_transformer ${catkin_LIBRARIES})

add_executable(VFH src/VFH.cpp)
target_link_libraries(VFH ${catkin_LIBRARIES} ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so)

add_executable(gripper_client src/gripper_client.cpp)
target_link_libraries(gripper_client ${catkin_LIBRARIES})

add_executable(pick_place_copy src/pick_place_copy.cpp)
target_link_libraries(pick_place_copy ${catkin_LIBRARIES})

add_executable(goback src/goback.cpp)
target_link_libraries(goback ${catkin_LIBRARIES})

#add_executable(calibration_hand_eye src/calibration_hand_eye.cpp)
#target_link_libraries(calibration_hand_eye ${catkin_LIBRARIES})

#add_executable(calib_opencv src/calib_opencv.cpp)
#target_link_libraries(calib_opencv ${catkin_LIBRARIES})

#add_executable(Kinova_pick_place src/Kinova_pick_place.cpp)
#target_link_libraries(Kinova_pick_place ${catkin_LIBRARIES})

add_executable(sub_and_go src/sub_and_go.cpp)
target_link_libraries(sub_and_go ${catkin_LIBRARIES}  ${kinova_driver_LIBRARIES})

add_executable(sub_and_go2 src/sub_and_go2.cpp)
target_link_libraries(sub_and_go2 ${catkin_LIBRARIES}  ${kinova_driver_LIBRARIES})

add_executable(find_and_pub src/find_and_pub.cpp)
target_link_libraries(find_and_pub ${catkin_LIBRARIES}  ${kinova_driver_LIBRARIES})

#add_executable(dmp_subscribe src/dmp_subscribe.cpp)
#target_link_libraries(dmp_subscribe ${catkin_LIBRARIES}  ${kinova_driver_LIBRARIES})

roslint_cpp(src/cloud_transformer.cpp)


