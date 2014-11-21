/*
 * File:   AccTwistOperationTest.hpp
 * Author: azamat
 *
 * Created on Jan 31, 2013, 1:21:58 PM
 */

#ifndef ACCTWISTOPERATIONTEST_HPP
#define	ACCTWISTOPERATIONTEST_HPP

#include <cppunit/extensions/HelperMacros.h>
#include <kdl_extensions/functionalcomputation_kdl.hpp>

class AccTwistOperationTest : public CPPUNIT_NS::TestFixture {
    CPPUNIT_TEST_SUITE(AccTwistOperationTest);

    CPPUNIT_TEST(testTransformAccTwist);
    CPPUNIT_TEST(testFailedTransformAccTwist);

    CPPUNIT_TEST_SUITE_END();

public:
    AccTwistOperationTest();
    virtual ~AccTwistOperationTest();
    void setUp();
    void tearDown();

private:
    KDL::Tree testTree;
    kdle::SegmentState a_segmentState;
    kdle::JointState a_jointState;
    
    void testTransformAccTwist();
    void testFailedTransformAccTwist();
};

#endif	/* ACCTWISTOPERATIONTEST_HPP */

