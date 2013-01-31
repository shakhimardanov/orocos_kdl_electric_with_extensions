/*
 * File:   PoseOperationTest.hpp
 * Author: azamat
 *
 * Created on Jan 24, 2013, 2:05:59 PM
 */

#ifndef POSEOPERATIONTEST_HPP
#define	POSEOPERATIONTEST_HPP

#include <cppunit/extensions/HelperMacros.h>
#include <kdl_extensions/functionalcomputation_kdltypes.hpp>

class PoseOperationTest : public CPPUNIT_NS::TestFixture {
    CPPUNIT_TEST_SUITE(PoseOperationTest);

    CPPUNIT_TEST(testTransformPose);
    CPPUNIT_TEST(testFailedTransformPose);

    CPPUNIT_TEST_SUITE_END();

public:
    PoseOperationTest();
    virtual ~PoseOperationTest();
    void setUp();
    void tearDown();

private:
    
    
    void testTransformPose();
    void testFailedTransformPose();
};

#endif	/* POSEOPERATIONTEST_HPP */

