cmake_minimum_required(VERSION 3.0.2)
project(encoder_calc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  nav_msgs
  sensor_msgs
  message_filters
)
list(APPEND CMAKE_MODULE_PATH /usr/share/cmake/geographiclib) # for ubuntu 18.04
find_package (GeographicLib REQUIRED)
find_package(Eigen3 REQUIRED)

message(Eigen: ${EIGEN3_INCLUDE_DIR})


generate_messages(
        DEPENDENCIES
        geometry_msgs
)

catkin_package(
        CATKIN_DEPENDS geometry_msgs
        DEPENDS EIGEN3
)
add_executable(${PROJECT_NAME} src/main.cpp  )
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})



include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
