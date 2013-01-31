/*
 * File:   TwistOperationTest.cpp
 * Author: azamat
 *
 * Created on Jan 31, 2013, 1:15:59 PM
 */

#include "twistOperationTest.hpp"


CPPUNIT_TEST_SUITE_REGISTRATION(TwistOperationTest);

TwistOperationTest::TwistOperationTest()
{
}

TwistOperationTest::~TwistOperationTest()
{
}

void TwistOperationTest::setUp()
{
}

void TwistOperationTest::tearDown()
{
}

void TwistOperationTest::testMethod()
{
    KDL::Segment testSegment("TestSegment");
    KDL::Tree testTree("TestTreeRoot");
    testTree.addSegment(testSegment, "TestTreeRoot");

    kdle::SegmentState a_segmentState, a_segmentState1;
    kdle::JointState a_jointState;
    KDL::SegmentMap::const_iterator segmentId = testTree.getRootSegment();
    kdle::transform<kdle::tree_iterator, kdle::twist> a_operation;

    a_segmentState1 = a_operation(segmentId, a_jointState, a_segmentState);
    CPPUNIT_ASSERT(a_segmentState == a_segmentState1);
}

void TwistOperationTest::testFailedMethod()
{
    CPPUNIT_ASSERT(false);
}

