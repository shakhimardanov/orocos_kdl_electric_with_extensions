cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
project(kdl_extensions)
SET(CMAKE_VERBOSE_MAKEFILE ON)
SET(CMAKE_CXX_FLAGS "-O0 -std=gnu++11")
#add_definitions(-DCHECK)
#add_definitions(-DOUTPUT_CORRECT)
#add_definitions(-DOUTPUT_WRONG)
#INCLUDE_DIRECTORIES(/usr/local/include/graphviz)
#LINK_DIRECTORIES(/usr/local/lib/graphviz)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} include/kdl_extensions/functionalcomputation.hpp
                    include/kdl_extensions/computationalstate_kdl.hpp src/computationalstate_kdl.cpp
                    include/kdl_extensions/functionalcomputation_kdl.hpp
                    include/kdl_extensions/chain_geometric_primitives.hpp
                    )

rosbuild_add_compile_flags(${PROJECT_NAME} "-O0")

target_link_libraries(${PROJECT_NAME} orocos-kdl geometric_semantics_kdl geometric_semantics)
#rosbuild_link_boost(${PROJECT_NAME} regex system serialization)

rosbuild_add_executable(compositiontest examples/compositiontest.cpp)
rosbuild_add_executable(chain_composition_geometricsemantics examples/chain_composition_geometricsemantics.cpp)
rosbuild_add_executable(composition_geometricsemantics examples/composition_geometricsemantics.cpp)
rosbuild_add_executable(composition_geometricsemantics_ver2 examples/composition_geometricsemantics_ver2.cpp)
rosbuild_add_executable(composition_geometricsemantics_compact examples/composition_geometricsemantics_compact.cpp)
rosbuild_add_compile_flags(${PROJECT_NAME} "-O0")

target_link_libraries(compositiontest ${PROJECT_NAME} graph gvc)
target_link_libraries(chain_composition_geometricsemantics ${PROJECT_NAME})
target_link_libraries(composition_geometricsemantics ${PROJECT_NAME})
target_link_libraries(composition_geometricsemantics_ver2 ${PROJECT_NAME})
target_link_libraries(composition_geometricsemantics_compact ${PROJECT_NAME})


ADD_SUBDIRECTORY( tests )
