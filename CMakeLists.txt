cmake_minimum_required(VERSION 3.0.2)
project(trajectory_prediction)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(Eigen3)
find_package(OsqpEigen)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  custom_trajectory_msgs
  OsqpEigen
  osqp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs sensor_msgs  custom_trajectory_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  #${Eigen_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

#
# MPC tracker
#
add_library(traj_predictor
  src/traj_predictor.cpp 
)
add_dependencies(traj_predictor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(traj_predictor ${catkin_LIBRARIES} osqp::osqp)

add_executable(traj_predictor_node
  src/traj_predictor_node.cpp
)
add_dependencies(traj_predictor_node traj_predictor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(traj_predictor_node traj_predictor ${catkin_LIBRARIES} OsqpEigen::OsqpEigen osqp::osqp)