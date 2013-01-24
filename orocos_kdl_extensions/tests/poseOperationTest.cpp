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
    a_operation = new kdle::transform<kdle::tree_iterator, kdle::pose>;
    a_segmentState = new KDL::SegmentState;
    a_jointState = new KDL::JointState;
    
}

void PoseOperationTest::tearDown()
{
    delete a_operation;
    delete a_segmetState;
    delete a_jointState;
}

void PoseOperationTest::testTransformPose()
{
    a_segmentState = a_operation(segmentId, a_jointState, a_segmentState);
    CPPUNIT_ASSERT(true);
}

void PoseOperationTest::testFailedTransformPose()
{
    CPPUNIT_ASSERT(false);
}

