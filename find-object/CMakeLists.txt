cmake_minimum_required(VERSION 2.8.5)

# Detect if it is called by catkin
IF(CATKIN_TOPLEVEL OR CATKIN_BUILD_BINARY_PACKAGE OR CATKIN_SKIP_TESTING OR CATKIN_ENABLE_TESTING OR CATKIN_DEVEL_PREFIX)
  SET(CATKIN_BUILD TRUE)
ENDIF(CATKIN_TOPLEVEL OR CATKIN_BUILD_BINARY_PACKAGE OR CATKIN_SKIP_TESTING OR CATKIN_ENABLE_TESTING OR CATKIN_DEVEL_PREFIX)

MESSAGE(STATUS "CATKIN_BUILD=${CATKIN_BUILD}")

IF(NOT CATKIN_BUILD)
   #Standalone build
   PROJECT( Find-Object )
ELSE()
   #ROS catkin build
   PROJECT( find_object_2d )
ENDIF()

# Catkin doesn't support multiarch library path, 
# fix to "lib" if not set by user.
IF(NOT DEFINED CMAKE_INSTALL_LIBDIR)
   set(CMAKE_INSTALL_LIBDIR "lib")
ENDIF(NOT DEFINED CMAKE_INSTALL_LIBDIR)

INCLUDE(GNUInstallDirs)

####### local cmake modules #######
SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")

# Policy CMP0043 introduced in cmake version 3.0 IGNORES the use of COMPILE_DEFINITIONS suffixed variables, e.g. COMPILE_DEFINITIONS_DEBUG
# Set to OLD behavior until minimum cmake version >= 2.8.10 (version that COMPILE_DEFINITIONS can be set by generator expressions instead)
if (POLICY CMP0043)
   cmake_policy(SET CMP0043 OLD)
endif (POLICY CMP0043)
# Policy CMP0042 introduced in cmake version 3.0 enables the use of @rpath in an install name via MACOSX_RPATH by default
# Set to OLD behavior so that all versions use the same behavior, or until minimum cmake version >= 2.8.12 (version where @rpath is available)
if (POLICY CMP0042)
   cmake_policy(SET CMP0042 OLD)
endif (POLICY CMP0042)

#######################
# VERSION
#######################
SET(PROJECT_VERSION "0.6.3")
SET(PROJECT_PREFIX find_object)

STRING(REGEX MATCHALL "[0-9]" PROJECT_VERSION_PARTS "${PROJECT_VERSION}")
LIST(GET PROJECT_VERSION_PARTS 0 PROJECT_VERSION_MAJOR)
LIST(GET PROJECT_VERSION_PARTS 1 PROJECT_VERSION_MINOR)
LIST(GET PROJECT_VERSION_PARTS 2 PROJECT_VERSION_PATCH)

ADD_DEFINITIONS(-DPROJECT_PREFIX="${PROJECT_PREFIX}")
ADD_DEFINITIONS(-DPROJECT_VERSION="${PROJECT_VERSION}")
ADD_DEFINITIONS(-DPROJECT_NAME="${PROJECT_NAME}")

####### DEPENDENCIES #######
FIND_PACKAGE(OpenCV 4.3 REQUIRED) # tested on 2.3.1

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

if(CMAKE_VERSION VERSION_LESS "2.8.11")
  # Add OpenCV headers location to your include paths
  include_directories(${OpenCV_INCLUDE_DIRS})
endif()


# For SuperPoint
SET(TORCH 0)
FIND_PACKAGE(Torch QUIET)
IF(TORCH_FOUND)
    MESSAGE(STATUS "Found Torch: ${TORCH_INCLUDE_DIRS}")
    SET(TORCH 1)
ENDIF(TORCH_FOUND)

# look for Qt5 before Qt4
FIND_PACKAGE(Qt5 COMPONENTS Widgets Core Gui Network QUIET)
IF(NOT Qt5_FOUND)
   FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui QtNetwork REQUIRED)
ENDIF(NOT Qt5_FOUND)
ADD_DEFINITIONS(-DQT_NO_KEYWORDS) # To avoid conflicts with boost signals used in ROS

FIND_PACKAGE(Tcmalloc QUIET)

FIND_PACKAGE(ZLIB REQUIRED QUIET)


SET(NONFREE 1)
IF(NOT (OPENCV_NONFREE_FOUND OR OPENCV_XFEATURES2D_FOUND))
   SET(NONFREE 0)
ELSEIF(OpenCV_VERSION VERSION_GREATER "3.4.2")
  FIND_FILE(OpenCV_MODULES_HPP opencv2/opencv_modules.hpp 
    PATHS ${OpenCV_INCLUDE_DIRS}
    NO_DEFAULT_PATH)
  FILE(READ ${OpenCV_MODULES_HPP} TMPTXT)
  STRING(FIND "${TMPTXT}" "#define OPENCV_ENABLE_NONFREE" matchres)
  IF(${matchres} EQUAL -1)
    SET(NONFREE 0)
  ENDIF(${matchres} EQUAL -1)
ENDIF()

CONFIGURE_FILE(Version.h.in ${PROJECT_SOURCE_DIR}/include/${PROJECT_PREFIX}/Version.h)


find_package(catkin REQUIRED COMPONENTS 
           cv_bridge roscpp rospy sensor_msgs std_msgs image_transport message_filters tf message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ObjectsStamped.msg
  DetectionInfo.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs std_msgs image_transport message_filters tf message_runtime
)

###########
## Build ##
###########
ADD_SUBDIRECTORY( src )

#############
## Install ##
#############
## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/find_object_3d_kinect2.launch
  launch/find_object_3d_zed.launch
  launch/find_object_2d.launch
  launch/find_object_3d.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

