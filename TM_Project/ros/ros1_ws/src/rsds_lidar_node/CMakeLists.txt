cmake_minimum_required(VERSION 2.8.3)
project(rsds_lidar_node)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_conversions
  pcl_ros
  tf2_ros
  tf2
  tf2_geometry_msgs
)

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} /opt/ipg/carmaker/linux64-10.0.1/include)

## Add source executable, InfoFile lib(s) path
link_directories(${PROJECT_NAME} /opt/ipg/carmaker/linux64-10.0.1/lib)

find_library(ifile_lib libinfofile.a /opt/ipg/carmaker/linux64-10.0.1/lib/)
find_library(cm_lib libcarmaker.a /opt/ipg/carmaker/linux64-10.0.1/lib/)

add_executable(${PROJECT_NAME} src/rsds_lidar_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${ifile_lib} ${cm_lib} ${CMAKE_DL_LIBS})

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})