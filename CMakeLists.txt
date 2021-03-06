cmake_minimum_required(VERSION 2.8.3)
project(model_joint_plugin)

add_definitions("-std=c++11")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)

find_package(catkin QUIET)

find_package(Protobuf REQUIRED)
if(${PROTOBUF_PROTOC_EXECUTABLE} MATCHES "PROTOBUF_PROTOC_EXECUTABLE-NOTFOUND")
    message(FATAL_ERROR "\nPlease install protobuf-compiler :\n    sudo apt install protobuf-compiler")    
endif()

find_package(GAZEBO REQUIRED)
if(GAZEBO_VERSION)
  if(${GAZEBO_VERSION} GREATER 6)
      add_definitions(-DGAZEBO_GREATER_6)
  endif()
endif()


set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set(msgs proto/joint_state.proto)

if(catkin_FOUND)
    catkin_package(LIBRARIES joint_state_msgs model_joint_plugin INCLUDE_DIRS include)
else()
    set(CATKIN_PACKAGE_LIB_DESTINATION lib)
    set(CATKIN_PACKAGE_LIB_DESTINATION lib)
    set(CATKIN_PACKAGE_BIN_DESTINATION bin)
    set(CATKIN_PACKAGE_INCLUDE_DESTINATION include/${PROJECT_NAME})
endif()

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(joint_state_msgs ${PROTO_SRCS})
target_link_libraries(joint_state_msgs ${PROTOBUF_LIBRARY})

include_directories(
  include
  ${PROJECT_BINARY_DIR} # for protobuf
  ${GAZEBO_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)


add_library(model_joint_plugin
  src/model_joint_plugin.cc
)

add_executable(model_joint_listener_test src/model_joint_listener_test.cc)

add_dependencies(model_joint_plugin joint_state_msgs)
add_dependencies(model_joint_listener_test joint_state_msgs)

target_link_libraries(model_joint_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Protobuf_LIBRARIES}
  joint_state_msgs
)
target_link_libraries(model_joint_listener_test
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Protobuf_LIBRARIES}
  joint_state_msgs
)

install(TARGETS model_joint_plugin model_joint_listener_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN ".svn" EXCLUDE
)

install(FILES
  ${PROTO_HDRS}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

