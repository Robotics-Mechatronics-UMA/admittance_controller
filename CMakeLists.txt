cmake_minimum_required(VERSION 2.8.3)
project(admittance_controller)

add_compile_options(-std=c++11)

set(${PROJECT_NAME}_DEPENDS
  #Package dependencies
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  ${${PROJECT_NAME}_DEPENDS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_DEPENDS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Create libraries
add_library(admittance_model src/admittance_model.cpp)
target_link_libraries(admittance_model ${catkin_LIBRARIES})

add_library(admittance_ros_interface src/admittance_ros_interface.cpp)
target_link_libraries(admittance_ros_interface ${catkin_LIBRARIES} admittance_model)

## Create executables (nodes)
add_executable(admittance_ros_node src/admittance_ros_node.cpp)
target_link_libraries(admittance_ros_node ${catkin_LIBRARIES} admittance_ros_interface)

#Dummy node
add_executable(dummy_test_node src/DummyTest.cpp)
target_link_libraries(dummy_test_node ${catkin_LIBRARIES})



