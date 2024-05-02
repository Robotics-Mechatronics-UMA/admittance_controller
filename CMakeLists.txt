cmake_minimum_required(VERSION 2.8.3)
project(admittance_controller)

## Buscar paquetes requeridos
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  dynamic_reconfigure
)

## Incluir la biblioteca Eigen
find_package(Eigen3 REQUIRED)

## Generar opciones de reconfiguración dinámica
generate_dynamic_reconfigure_options(
  cfg/params.cfg
)

## Declarar el paquete de catkin
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs dynamic_reconfigure
)

## Incluir directorios
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}  # Incluir directorio de Eigen
)

## Crear bibliotecas
add_library(admittance_model src/admittance_model.cpp)
target_link_libraries(admittance_model ${catkin_LIBRARIES})

add_library(admittance_ros_interface src/admittance_ros_interface.cpp)
target_link_libraries(admittance_ros_interface ${catkin_LIBRARIES} admittance_model)

## Crear ejecutables (nodos)
add_executable(admittance_ros_node src/admittance_ros_node.cpp)
target_link_libraries(admittance_ros_node ${catkin_LIBRARIES} admittance_ros_interface)

# Nodo de prueba
add_executable(dummy_test_node src/DummyTest.cpp)
target_link_libraries(dummy_test_node ${catkin_LIBRARIES})
