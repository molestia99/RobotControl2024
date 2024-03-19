cmake_minimum_required(VERSION 2.8.3)
project(pongbot_q_robotcontrol2024)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
  sensor_msgs		
)

## System dependencies are found with CMake's conventions
find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})

find_package (RBDL REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (osqp REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES pongbot_q_robotcontrol2024
   DEPENDS gazebo_ros roscpp rospy std_msgs sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${RBDL_INCLUDE_DIR} 
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} 
   #src/pongbotq_plugin.cc
   src/pongbotq_plugin.cc  
   
   #src/CRobot/CRobot.cpp
#   src/CRobot/nrutil/nrutil.cpp
   #src/CRobot/nrutil/nrlin.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES} 
  ${RBDL_LIBRARY} 
  ${RBDL_URDFReader_LIBRARY}
)

target_link_libraries(${PROJECT_NAME} PRIVATE osqp::osqp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
