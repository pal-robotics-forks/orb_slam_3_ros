cmake_minimum_required(VERSION 3.0.2)
project(orb_slam3_ros)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

# Check for c++11 support
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
IF(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
ELSE()
  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/orb_slam3/cmake_modules)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)
disable_pal_flags()

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

catkin_package(
  LIBRARIES {PROJECT_NAME} libDBoW2 libg2o
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
)

include_directories(
  ${PROJECT_SOURCE_DIR}/orb_slam3
  ${PROJECT_SOURCE_DIR}/orb_slam3/include
  ${PROJECT_SOURCE_DIR}/ros/include
)

include_directories(
  SYSTEM
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o)
add_subdirectory(${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2)

add_library(${PROJECT_NAME} SHARED
orb_slam3/src/CameraModels/KannalaBrandt8.cpp
orb_slam3/src/CameraModels/Pinhole.cpp
orb_slam3/src/Atlas.cc
orb_slam3/src/Converter.cc
orb_slam3/src/Frame.cc
orb_slam3/src/FrameDrawer.cc
orb_slam3/src/G2oTypes.cc
orb_slam3/src/ImuTypes.cc
orb_slam3/src/Initializer.cc
orb_slam3/src/KeyFrame.cc
orb_slam3/src/KeyFrameDatabase.cc
orb_slam3/src/LocalMapping.cc
orb_slam3/src/LoopClosing.cc
orb_slam3/src/Map.cc
orb_slam3/src/MapPoint.cc
orb_slam3/src/MLPnPsolver.cpp
orb_slam3/src/OptimizableTypes.cpp
orb_slam3/src/Optimizer.cc
orb_slam3/src/ORBextractor.cc
orb_slam3/src/ORBmatcher.cc
orb_slam3/src/PnPsolver.cc
orb_slam3/src/Sim3Solver.cc
orb_slam3/src/System.cc
orb_slam3/src/Tracking.cc
orb_slam3/src/TwoViewReconstruction.cc
)
add_dependencies (${PROJECT_NAME} g2o DBoW2)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
DBoW2
g2o
)

add_library(wrappers_library SHARED
  ros/src/wrappers/base_wrapper.cpp
  ros/src/wrappers/stereo_wrapper.cpp
  ros/src/wrappers/converter.cpp
)
add_dependencies(wrappers_library ${PROJECT_NAME})
target_link_libraries(wrappers_library
  ${PROJECT_NAME})
  
add_executable(stereo_slam_node ros/src/nodes/stereo_slam_node.cpp)
add_dependencies(stereo_slam_node wrappers_library)
target_link_libraries(stereo_slam_node
  wrappers_library
  ${catkin_LIBRARIES})

