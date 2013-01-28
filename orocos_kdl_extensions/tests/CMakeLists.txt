# INCLUDE_DIRECTORIES(/usr/include/cppunit)
# LINK_DIRECTORIES(/usr/lib/cppunit)

rosbuild_add_executable(poseoperation4treelink-test poseoperation4treelink-test.cpp)
target_link_libraries(poseoperation4treelink-test ${PROJECT_NAME})

rosbuild_add_executable(poseOperationTest testrunner.cpp poseOperationTest.cpp )
target_link_libraries(poseOperationTest ${PROJECT_NAME} cppunit)