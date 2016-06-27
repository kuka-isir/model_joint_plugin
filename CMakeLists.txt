cmake_minimum_required(VERSION 2.8.3)
project(model_joint_plugin)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE Release)


find_package(catkin REQUIRED cmake_modules)

#find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})
#include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

find_package(orocos_kdl REQUIRED)
find_package(Eigen REQUIRED)
find_package(GAZEBO REQUIRED)
find_package(Protobuf REQUIRED)

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

set (msgs
  proto/joint_state.proto
)

catkin_package(LIBRARIES joint_state_msgs model_joint_plugin
INCLUDE_DIRS include)

set(CMAKE_POSITION_INDEPENDENT_CODE "TRUE")
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(joint_state_msgs SHARED ${PROTO_SRCS})
target_link_libraries(joint_state_msgs ${PROTOBUF_LIBRARY})
set_source_files_properties(${PROTO_SRCS} ${PROTO_HDRS} PROPERTIES
                            GENERATED TRUE)

add_custom_target(copy_joint_state_msgs DEPENDS joint_state_msgs)
set(run_cmd 0)
foreach(header ${PROTO_HDRS})
    if(NOT EXISTS ${header})
        set(destination ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME})
        message( "${header} --> ${destination}")
        add_custom_command(TARGET copy_joint_state_msgs PRE_BUILD
                        COMMAND ${CMAKE_COMMAND} -E
                            copy ${header} ${destination}
                        MAIN_DEPENDENCY copy_joint_state_msgs)
        set(run_cmd 1)
    endif()

endforeach()

if(${run_cmd})
    add_custom_target(run ALL
            DEPENDS copy_joint_state_msgs)
endif()

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})

include_directories(
  include
  ${PROJECT_BINARY_DIR}/../ # for protobuf
  ${GAZEBO_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

# NOTE : We use add_library otherwise orocos adds -gnulinux or -xenomai
# And that causes problems with <gazebo> load plugin "libmodel_joint_plugin.so"
# But also need to export the lib (with catkin package)
add_library(model_joint_plugin
  src/model_joint_plugin.cc
)

# orocos_component(gz_model_remote_interface
#   src/gz_model_remote_interface.cc
# )
# target_link_libraries(gz_model_remote_interface
#   ${catkin_LIBRARIES}
#   ${GAZEBO_LIBRARIES}
#   ${Protobuf_LIBRARIES}
#   joint_state_msgs
#   ${orocos_kdl_LIBRARIES}
# )
# add_dependencies(gz_model_remote_interface joint_state_msgs)

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

install(DIRECTORY ${PROJECT_SOURCE_DIR}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN ".svn" EXCLUDE
)

install(FILES
  ${PROTO_HDRS}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# orocos_install_headers(DIRECTORY include)
# orocos_install_headers(${PROTO_HDRS})
#
# orocos_generate_package(INCLUDE_DIRS include)
