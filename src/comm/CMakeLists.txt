cmake_minimum_required(VERSION 3.0.2)
project(comm)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  msg_collection
)
find_package(yaml-cpp REQUIRED)

catkin_package(
 INCLUDE_DIRS include
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)


set (CMAKE_CXX_FLAGS "${REDISCPP_FLAGS}")
find_package(Boost REQUIRED COMPONENTS thread system iostreams)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
set (LIB_BOOST
    ${LIBRARIES}
    ${Boost_LIBRARIES}
)
add_library(lib_redis STATIC IMPORTED)
set_target_properties(lib_redis PROPERTIES
IMPORTED_LOCATION /usr/local/lib/libredis-cpp.a
INTERFACE_INCLUDE_DIRECTORIES /usr/local/include
)

add_executable(motor src/motor.cpp src/rs232.c)
target_link_libraries(motor ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})