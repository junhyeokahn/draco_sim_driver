cmake_minimum_required(VERSION 3.0.2)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif(NOT CMAKE_BUILD_TYPE)

project(rolling_joint)

include_directories("/usr/local/include")
link_directories("/usr/local/lib")
list(APPEND CMAKE_MODULE_PATH "/home/apptronik/Repository/PnC/cmake")

find_package(catkin REQUIRED COMPONENTS cortex_utils message_generation message_runtime nodelet pluginlib roscpp roslib rt_utils tf2_eigen nodelet)
find_package(Boost QUIET REQUIRED COMPONENTS system thread filesystem)
find_package(DracoPnC REQUIRED)
find_package(DART 6.10 REQUIRED COMPONENTS utils-urdf gui-osg CONFIG)
include_directories(${DART_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cortex_utils message_generation message_runtime nodelet pluginlib roscpp roslib rt_utils tf2_eigen
  )

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${DRACO_PNC_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++14 -fPIC -Wno-unused-result -Wno-overflow -Dunix -rdynamic")

FIND_PACKAGE(PkgConfig)
PKG_CHECK_MODULES(EIGEN3 REQUIRED eigen3)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/rolling_joint.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
                                      ${DART_LIBRARIES}
                                      ${DRACO_PNC_LIBRARIES})

#message(${PNC_INCLUDE_DIRS})
#message(${PNC_LIBRARIES})

