cmake_minimum_required(VERSION 3.0.2)
project(admittance_ros_interface)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  # dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)

set(${PROJECT_NAME}_INCLUDE_DIRS
  #Include folders
  include
  ${EIGEN3_INCLUDE_DIRS}
)

find_package(catkin REQUIRED COMPONENTS
  ${${PROJECT_NAME}_DEPENDS}
)

#What to export
catkin_package(
  INCLUDE_DIRS ${catkin_INCLUDE_DIRS} ${${PROJECT_NAME}_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_DEPENDS}
)

include_directories( ${catkin_INCLUDE_DIRS} ${${PROJECT_NAME}_INCLUDE_DIRS} )

##Create libraries
add_library(admittance_model src/admittance_model.cpp)
add_dependencies(admittance_model ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(admittance_model ${catkin_LIBRARIES})

add_library(admittance_ros_interface src/admittance_ros_interface.cpp)
add_dependencies(admittance_ros_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(admittance_ros_interface ${catkin_LIBRARIES} admittance_model)


##Create executables (nodes)
#Admittance node
add_executable(admittance_ros_node src/admittance_ros_node.cpp)
target_link_libraries(admittance_ros_node ${catkin_LIBRARIES})
add_dependencies(admittance_ros_node ${catkin_EXPORTED_TARGETS})

#Dummy node
add_executable(dummy_test_node src/DummyTest.cpp)
target_link_libraries(dummy_test_node ${catkin_LIBRARIES})
add_dependencies(dummy_test_node ${catkin_EXPORTED_TARGETS})

