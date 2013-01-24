# INCLUDE_DIRECTORIES(/usr/include/cppunit)
# LINK_DIRECTORIES(/usr/lib/cppunit)

rosbuild_add_executable(poseoperation4treelink-test poseoperation4treelink-test.cpp)
target_link_libraries(poseoperation4treelink-test ${PROJECT_NAME})

rosbuild_add_executable(testrunner testrunner.cpp)
target_link_libraries(testrunner ${PROJECT_NAME} cppunit)