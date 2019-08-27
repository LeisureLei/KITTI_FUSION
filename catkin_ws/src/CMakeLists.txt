cmake_minimum_required(VERSION 2.8.3)
project(vlo)

if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  std_msgs
 )

find_package(PCL 1.7 REQUIRED)

find_package(OpenCV  REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS  roscpp  std_msgs
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS include
)


add_executable(laserhandle src/laserhandle.cpp)
target_link_libraries(laserhandle ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(lidtocam src/lidtocam.cpp)
target_link_libraries(lidtocam ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
