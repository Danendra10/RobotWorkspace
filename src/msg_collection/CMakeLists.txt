cmake_minimum_required(VERSION 3.0.2)
project(msg_collection)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
    FILES
    stmToPc.msg
    encMotor.msg
)

add_service_files(
  FILES
  ThresholdVision.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iris_msgs
#  CATKIN_DEPENDS std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)