/*
 * File:   TwistOperationTest.hpp
 * Author: azamat
 *
 * Created on Jan 31, 2013, 1:16:00 PM
 */

#ifndef TWISTOPERATIONTEST_HPP
#define	TWISTOPERATIONTEST_HPP

#include <cppunit/extensions/HelperMacros.h>
#include <kdl_extensions/functionalcomputation_kdl.hpp>

class TwistOperationTest : public CPPUNIT_NS::TestFixture {
    CPPUNIT_TEST_SUITE(TwistOperationTest);

    CPPUNIT_TEST(testTransformTwist);
    CPPUNIT_TEST(testFailedTransformTwist);

    CPPUNIT_TEST_SUITE_END();

public:
    TwistOperationTest();
    virtual ~TwistOperationTest();
    void setUp();
    void tearDown();

private:
    KDL::Tree testTree;
    kdle::SegmentState a_segmentState;
    kdle::JointState a_jointState;

    void testTransformTwist();
    void testFailedTransformTwist();
};

#endif	/* TWISTOPERATIONTEST_HPP */

