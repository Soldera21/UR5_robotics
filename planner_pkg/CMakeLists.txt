cmake_minimum_required(VERSION 2.8.3)
project(planner_pkg)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++17")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs sensor_msgs control_msgs ros_impedance_controller message_generation)
find_package(Eigen3 3.3 REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY
  msg
   FILES
   legoDetection.msg
   legoGroup.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   geometry_msgs
 )

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include/planner_pkg/
  LIBRARIES planner_pkg
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(planner src/planner.cpp src/movement.cpp src/kinematics.cpp)

#add_library(planner_pkg src/kinematics.cpp src/movement.cpp src/planner.cpp)
target_link_libraries(planner ${catkin_LIBRARIES} ${catkin_INCLUDE_DIRS})

install(TARGETS planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE
)
