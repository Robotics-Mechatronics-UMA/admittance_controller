cmake_minimum_required(VERSION 3.0.2)
project(robot_delta)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  # rosserial_arduino
  dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/params.cfg
)

###################################
## catkin specific configuration ##
###################################
# find_package(rosserial_arduino REQUIRED)

## Generate libraries for Arduino node
# rosserial_generate_ros_lib(
#   PACKAGE rosserial_arduino
#   SCRIPT make_libraries.py
# )

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs dynamic_reconfigure #rosserial_arduino 
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  include
)

## Declare a C++ executable for AdmittanceController
add_executable(admittance_controller_node src/AdmittanceController.cpp)
target_link_libraries(admittance_controller_node ${catkin_LIBRARIES})
add_dependencies(admittance_controller_node ${catkin_EXPORTED_TARGETS})

#Declaramos un ejecutable para ForceSensor de pruebas
add_executable(force_sensor_node src/ForceSensor.cpp)
target_link_libraries(force_sensor_node ${catkin_LIBRARIES})
add_dependencies(force_sensor_node ${catkin_EXPORTED_TARGETS})

# ## Declare an executable for Arduino node (ForceSensor.ino)
# add_executable(force_sensor_node src/ForceSensor.ino)
# target_link_libraries(force_sensor_node ${catkin_LIBRARIES})
# add_dependencies(force_sensor_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

# ## Incluye el paquete de ros_lib en la construcción
# include_directories(${catkin_INCLUDE_DIRS} ${ROSSERIAL_ARDUINO_PACKAGE_PATH}/src)
