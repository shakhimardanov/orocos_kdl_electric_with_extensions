/*
 * File:   AccTwistOperationTest.cpp
 * Author: azamat
 *
 * Created on Jan 31, 2013, 1:21:58 PM
 */

#include "accTwistOperationTest.hpp"
#include "kdl_extensions/functionalcomputation_kdltypes.hpp"


CPPUNIT_TEST_SUITE_REGISTRATION(AccTwistOperationTest);

AccTwistOperationTest::AccTwistOperationTest()
{
}

AccTwistOperationTest::~AccTwistOperationTest()
{
}

void AccTwistOperationTest::setUp()
{
}

void AccTwistOperationTest::tearDown()
{
}

void AccTwistOperationTest::testMethod()
{
  KDL::Segment testSegment("TestSegment");
    KDL::Tree testTree("TestTreeRoot");
    testTree.addSegment(testSegment,"TestTreeRoot");

    kdle::SegmentState a_segmentState, a_segmentState1;
    kdle::JointState a_jointState;
    KDL::SegmentMap::const_iterator segmentId = testTree.getRootSegment();
    kdle::transform<kdle::tree_iterator, kdle::accTwist> a_operation;

    a_segmentState1 = a_operation(segmentId, a_jointState, a_segmentState);
    CPPUNIT_ASSERT(a_segmentState == a_segmentState1);
}

void AccTwistOperationTest::testFailedMethod()
{
    CPPUNIT_ASSERT(false);
}

