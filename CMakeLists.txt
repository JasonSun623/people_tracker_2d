cmake_minimum_required(VERSION 2.8.3)
project(people_tracker_2d)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tf
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cmake_modules
  message_generation)

find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)

add_compile_options(-std=c++11)

catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS roscpp roslib tf std_msgs sensor_msgs geometry_msgs nav_msgs message_runtime
DEPENDS EIGEN3)

include_directories(include ${catkin_INCLUDE_DIRS}
                            ${EIGEN3_INCLUDE_DIRS}
                            ${GTSAM_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS})

link_directories(${csm_LIBRARY_DIRS})

# Libraries
add_library(ObjectDetector_2d src/object_detector.cpp)
add_library(TrackedObject src/tracked_object.cpp)
add_library(KalmanFilter_2d src/kalman_filter.cpp)
add_library(people_tracker_2d_lib src/people_tracker_2d.cpp)

# Link libraries
target_link_libraries(people_tracker_2d_lib ${catkin_LIBRARIES} ObjectDetector_2d
                      TrackedObject KalmanFilter_2d ${EIGEN3_LIBRARIES})

# Executables
add_executable(people_tracker_2d src/people_tracker_2d.cpp)

# Link libraries
target_link_libraries(people_tracker_2d people_tracker_2d_lib)

# Install

install(DIRECTORY
  launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
