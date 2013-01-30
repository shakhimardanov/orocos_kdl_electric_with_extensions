/*
 * File:   PoseOperationTest.cpp
 * Author: azamat
 *
 * Created on Jan 24, 2013, 2:05:58 PM
 */

#include "poseOperationTest.hpp"


CPPUNIT_TEST_SUITE_REGISTRATION(PoseOperationTest);

PoseOperationTest::PoseOperationTest()
{
    
}

PoseOperationTest::~PoseOperationTest()
{

}

void PoseOperationTest::setUp()
{


}

void PoseOperationTest::tearDown()
{

}

void PoseOperationTest::testTransformPose()
{
    KDL::Segment testSegment("TestSegment");
    KDL::Tree testTree("TestTreeRoot");
    testTree.addSegment(testSegment,"TestTreeRoot");

    KDL::SegmentState a_segmentState, a_segmentState1;
    KDL::JointState a_jointState;
    KDL::SegmentMap::const_iterator segmentId = testTree.getRootSegment();
    kdle::transform<kdle::tree_iterator, kdle::pose> a_operation;
    
    a_segmentState1 = a_operation(segmentId, a_jointState, a_segmentState);
    CPPUNIT_ASSERT(a_segmentState == a_segmentState1);
}

void PoseOperationTest::testFailedTransformPose()
{ KDL::SegmentState a_segmentState, a_segmentState1;

    CPPUNIT_ASSERT_ASSERTION_FAIL(CPPUNIT_ASSERT(a_segmentState == a_segmentState1));
}

