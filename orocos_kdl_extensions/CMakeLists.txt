cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

INCLUDE_DIRECTORIES(/usr/local/include/graphviz)
LINK_DIRECTORIES(/usr/local/lib/graphviz)

rosbuild_init()
project(kdl_extensions)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} include/kdl_extensions/functionalcomputation.hpp src/functionalcomputation.inl
                    include/kdl_extensions/functionalcomputation_kdltypes.hpp src/functionalcomputation_kdltypes.inl
                    include/kdl_extensions/treeid_vereshchagin_composable.hpp src/treeid_vereshchagin_composable.cpp
                    include/kdl_extensions/computationalstate_kdltypes.hpp src/computationalstate_kdltypes.cpp)

target_link_libraries(${PROJECT_NAME} orocos-kdl geometric_semantics_kdl)
rosbuild_add_executable(compositiontest examples/compositiontest.cpp)
target_link_libraries(compositiontest ${PROJECT_NAME} graph gvc)

ADD_SUBDIRECTORY( tests )
