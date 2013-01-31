# INCLUDE_DIRECTORIES(/usr/include/cppunit)
# LINK_DIRECTORIES(/usr/lib/cppunit)

rosbuild_add_executable(poseoperation4treelink-test poseoperation4treelink-test.cpp)
target_link_libraries(poseoperation4treelink-test ${PROJECT_NAME})

rosbuild_add_executable(twistoperation4treelink-test twistoperation4treelink-test.cpp)
target_link_libraries(twistoperation4treelink-test ${PROJECT_NAME})

rosbuild_add_executable(testSuite testrunner.cpp poseOperationTest.cpp twistOperationTest.cpp accTwistOperationTest.cpp
                        forceOperationTest.cpp)
target_link_libraries(testSuite ${PROJECT_NAME} cppunit)