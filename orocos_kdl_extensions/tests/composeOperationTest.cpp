/*
 * File:   ComposeOperationTest.cpp
 * Author: azamat
 *
 * Created on Feb 12, 2013, 1:00:41 PM
 */

#include "composeOperationTest.hpp"


CPPUNIT_TEST_SUITE_REGISTRATION(ComposeOperationTest);

ComposeOperationTest::ComposeOperationTest()
{
}

ComposeOperationTest::~ComposeOperationTest()
{
}

void ComposeOperationTest::setUp()
{
    KDL::Joint testJoint = KDL::Joint("TestJoint", KDL::Joint::RotZ, 1, 0, 0.01);
    KDL::Frame testFrame(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, -0.4, 0.0));
    KDL::Segment testSegment = KDL::Segment("TestSegment", testJoint, testFrame);
    KDL::RotationalInertia testRotInerSeg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0); //around symmetry axis of rotation
    double pointMass = 0.25; //in kg
    KDL::RigidBodyInertia testInerSegment(pointMass, KDL::Vector(0.0, -0.4, 0.0), testRotInerSeg);
    testSegment.setInertia(testInerSegment);

    testTree.addSegment(testSegment, "root");

    a_segmentState = kdle::SegmentState();
    a_jointState = kdle::JointState();

    a_operation1 = kdle::transform<kdle::tree_iterator, kdle::pose > ();
    a_operation2 = kdle::transform<kdle::tree_iterator, kdle::twist > ();
    a_operation3 = kdle::transform<kdle::tree_iterator, kdle::accTwist > ();
    a_operation4 = kdle::balance<kdle::tree_iterator, kdle::force > ();


}

void ComposeOperationTest::tearDown()
{
}

void ComposeOperationTest::testComposition()
{
    kdle::SegmentState a_segmentState1;
    kdle::SegmentState a_segmentState2;

    //    update a state by performing two consecutive operations on it
    //    in this case it is first pose transform and then twist transform
    a_segmentState1 = a_operation1(testTree.getSegment("TestSegment"), a_jointState, a_segmentState);
    a_segmentState2 = a_operation2(testTree.getSegment("TestSegment"), a_jointState, a_segmentState1);

    //    update a state applying composed operations on it
    //    in this case pose and twist transform operations are functionally composed
    a_segmentState1 = kdle::compose(a_operation2, a_operation1) (testTree.getSegment("TestSegment"), a_jointState, a_segmentState);

    //    the outcome of two versions  of updates should be the same
    CPPUNIT_ASSERT(a_segmentState2 == a_segmentState1);
}

void ComposeOperationTest::testFailedComposition()
{
    kdle::SegmentState a_segmentState1;
    kdle::SegmentState a_segmentState2;

    //    update a state by applying four simple operations consequently
    a_segmentState1 = a_operation1(testTree.getSegment("TestSegment"), a_jointState, a_segmentState);
    a_segmentState1 = a_operation2(testTree.getSegment("TestSegment"), a_jointState, a_segmentState1);
    a_segmentState1 = a_operation3(testTree.getSegment("TestSegment"), a_jointState, a_segmentState1);
    a_segmentState2 = a_operation4(testTree.getSegment("TestSegment"), a_jointState, a_segmentState1);

    //    update a state by applying complex operation, created by a nested composition
    a_segmentState1 = kdle::compose(kdle::compose(a_operation4, a_operation3), kdle::compose(a_operation2, a_operation1)) (testTree.getSegment("TestSegment"), a_jointState, a_segmentState);


    CPPUNIT_ASSERT(a_segmentState2 != a_segmentState1);
}

