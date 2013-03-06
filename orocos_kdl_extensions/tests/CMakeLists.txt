# INCLUDE_DIRECTORIES(/usr/include/cppunit)
# LINK_DIRECTORIES(/usr/lib/cppunit)

rosbuild_add_executable(fkposition-test fkposition.cpp)
target_link_libraries(fkposition-test ${PROJECT_NAME})

rosbuild_add_executable(fkposition-test2 fkposition2.cpp)
target_link_libraries(fkposition-test2 ${PROJECT_NAME} graph gvc)


rosbuild_add_executable(fkvelocity-test fkvelocity.cpp)
target_link_libraries(fkvelocity-test ${PROJECT_NAME})

rosbuild_add_executable(inversedynamics-test inversedynamics.cpp)
target_link_libraries(inversedynamics-test ${PROJECT_NAME})

rosbuild_add_executable(testSuite testrunner.cpp poseOperationTest.cpp twistOperationTest.cpp accTwistOperationTest.cpp
                        forceOperationTest.cpp composeOperationTest.cpp traverseOperationTest.cpp)
target_link_libraries(testSuite ${PROJECT_NAME} cppunit)