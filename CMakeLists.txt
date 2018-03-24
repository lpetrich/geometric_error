cmake_minimum_required(VERSION 2.8.3)
project(geometric_error)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  cmake_modules
)

find_package(Eigen REQUIRED)

# catkin_python_setup()


add_message_files(
  FILES
  TrackPoint.msg
  TrackedPoints.msg
  Error.msg
  ErrorInfo.msg
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
# INCLUDE_DIRS include
#  LIBRARIES geometric_error
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(error_control src/geometric_error.cpp)

target_link_libraries(error_control
  ${catkin_LIBRARIES}
 )