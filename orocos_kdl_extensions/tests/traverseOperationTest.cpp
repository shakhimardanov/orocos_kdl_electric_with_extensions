/*
 * File:   TraverseOperationTest.cpp
 * Author: azamat
 *
 * Created on Feb 12, 2013, 3:05:31 PM
 */

#include "traverseOperationTest.h"


CPPUNIT_TEST_SUITE_REGISTRATION(TraverseOperationTest);

TraverseOperationTest::TraverseOperationTest()
{
}

TraverseOperationTest::~TraverseOperationTest()
{
}

void TraverseOperationTest::setUp()
{
    KDL::Joint testJoint = KDL::Joint("TestJoint", KDL::Joint::RotZ, 1, 0, 0.01);
    KDL::Frame testFrame(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, -0.4, 0.0));
    KDL::Segment testSegment = KDL::Segment("TestSegment", testJoint, testFrame);
    KDL::RotationalInertia testRotInerSeg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0); //around symmetry axis of rotation
    double pointMass = 0.25; //in kg
    KDL::RigidBodyInertia testInerSegment(pointMass, KDL::Vector(0.0, -0.4, 0.0), testRotInerSeg);
    testSegment.setInertia(testInerSegment);

    testTree.addSegment(testSegment, "root");

    a_segmentState.resize(testTree.getSegments().size());
    a_jointState.resize(testTree.getSegments().size());

    a_operation1 = kdle::transform<kdle::tree_iterator, kdle::pose > ();
    a_operation2 = kdle::transform<kdle::tree_iterator, kdle::twist > ();

}

void TraverseOperationTest::tearDown()
{
}

void TraverseOperationTest::testTraverseOperation()
{
    std::vector<kdle::SegmentState> a_segmentState1;
    a_segmentState1.resize(testTree.getSegments().size());

    kdle::traverseGraph_ver2(testTree, kdle::compose(a_operation2, a_operation1), a_policy1)(a_jointState, a_segmentState, a_segmentState1);

    std::vector<kdle::SegmentState> a_segmentState2;
    a_segmentState2.resize(testTree.getSegments().size());

    a_segmentState2[0] = kdle::compose(a_operation2, a_operation1) (testTree.getSegment("root"), a_jointState[0], a_segmentState[0]);
    a_segmentState2[1] = kdle::compose(a_operation2, a_operation1) (testTree.getSegment("TestSegment"), a_jointState[1], a_segmentState2[0]);

    for (unsigned int i=0; i < testTree.getSegments().size(); i++ )
    {
        CPPUNIT_ASSERT(a_segmentState1[i] == a_segmentState2[i]);
    }
}

void TraverseOperationTest::testFailedTraverseOperation()
{
    CPPUNIT_ASSERT(false);
}

