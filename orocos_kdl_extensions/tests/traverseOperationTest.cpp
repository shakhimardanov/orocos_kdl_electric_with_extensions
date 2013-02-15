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
    testTree = KDL::Tree("Root");
    KDL::Joint testJoint0 = KDL::Joint("TestJoint0", KDL::Joint::RotZ, 1, 0, 0.01);
    KDL::Frame testFrame0(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, -0.4, 0.0));
    KDL::Segment testSegment0 = KDL::Segment("TestSegment0", testJoint0, testFrame0);
    KDL::RotationalInertia testRotInerSeg0(0.0, 0.0, 0.0, 0.0, 0.0, 0.0); //around symmetry axis of rotation
    double pointMass = 0.25; //in kg
    KDL::RigidBodyInertia testInerSegment0(pointMass, KDL::Vector(0.0, -0.4, 0.0), testRotInerSeg0);
    testSegment0.setInertia(testInerSegment0);

    testTree.addSegment(testSegment0, "Root");

    KDL::Joint testJoint1 = KDL::Joint("TestJoint1", KDL::Joint::RotZ, 1, 0, 0.01);
    KDL::Frame testFrame1(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, -0.4, 0.0));
    KDL::Segment testSegment1 = KDL::Segment("TestSegment1", testJoint1, testFrame1);
    KDL::RotationalInertia testRotInerSeg1(0.0, 0.0, 0.0, 0.0, 0.0, 0.0); //around symmetry axis of rotation

    KDL::RigidBodyInertia testInerSegment1(pointMass, KDL::Vector(0.0, -0.4, 0.0), testRotInerSeg1);
    testSegment1.setInertia(testInerSegment1);

    testTree.addSegment(testSegment1, "TestSegment0");


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

    a_jointState[0].q = KDL::PI / 3.0;
    a_jointState[0].qdot = 0.2;
    a_jointState[1].q = -KDL::PI / 3.0;
    a_jointState[1].qdot = -0.2;

    printf("initial pose x %f\n", a_segmentState[1].X.p[0]);
    printf("initial pose y %f\n", a_segmentState[1].X.p[1]);
    printf("initial pose z %f\n", a_segmentState[1].X.p[2]);
    printf("initial twist x %f\n", a_segmentState[1].Xdot.vel[0]);
    printf("initial twist y %f\n", a_segmentState[1].Xdot.vel[1]);
    printf("initial twist z %f\n", a_segmentState[1].Xdot.vel[2]);

    //    root segment is used in traversal but its state does not effect those of the segments of the physical structure (arm)
    kdle::traverseGraph_ver2(testTree, kdle::compose(a_operation2, a_operation1), a_policy1)(a_jointState, a_segmentState, a_segmentState1);

    printf("traversal: updated state of TestSegment0\n");
    printf("traversal: updated pose x %f\n", a_segmentState1[0].X.p[0]);
    printf("traversal: updated pose y %f\n", a_segmentState1[0].X.p[1]);
    printf("traversal: updated pose z %f\n", a_segmentState1[0].X.p[2]);
    printf("traversal: updated twist x %f\n", a_segmentState1[0].Xdot.vel[0]);
    printf("traversal: updated twist y %f\n", a_segmentState1[0].Xdot.vel[1]);
    printf("traversal: updated twist rot-z %f\n", a_segmentState1[0].Xdot.rot[2]);

    printf("traversal: updated state of TestSegment1\n");
    printf("traversal: updated pose x %f\n", a_segmentState1[1].X.p[0]);
    printf("traversal: updated pose y %f\n", a_segmentState1[1].X.p[1]);
    printf("traversal: updated pose z %f\n", a_segmentState1[1].X.p[2]);
    printf("traversal: updated twist x %f\n", a_segmentState1[1].Xdot.vel[0]);
    printf("traversal: updated twist y %f\n", a_segmentState1[1].Xdot.vel[1]);
    printf("traversal: updated twist rot-z %f\n", a_segmentState1[1].Xdot.rot[2]);


    std::vector<kdle::SegmentState> a_segmentState2;
    a_segmentState2.resize(testTree.getSegments().size());
    kdle::SegmentState tempInitialState;

    //    root segment is not condired in state-by-state composition below, since it is not part of the physical structure (arm)
    a_segmentState2[0] = kdle::compose(a_operation2, a_operation1) (testTree.getSegment("TestSegment0"), a_jointState[0], tempInitialState);
    a_segmentState2[1] = kdle::compose(a_operation2, a_operation1) (testTree.getSegment("TestSegment1"), a_jointState[1], a_segmentState2[0]);

    printf("traversal: updated state of TestSegment0\n");
    printf("without traversal: updated pose x %f\n", a_segmentState2[0].X.p[0]);
    printf("without traversal: updated pose y %f\n", a_segmentState2[0].X.p[1]);
    printf("without traversal: updated pose z %f\n", a_segmentState2[0].X.p[2]);
    printf("without traversal: updated twist x %f\n", a_segmentState2[0].Xdot.vel[0]);
    printf("without traversal: updated twist y %f\n", a_segmentState2[0].Xdot.vel[1]);
    printf("without traversal: updated twist rot-z %f\n", a_segmentState2[0].Xdot.rot[2]);

    CPPUNIT_ASSERT(a_segmentState1[0] == a_segmentState2[0]);

    printf("traversal: updated state of TestSegment1\n");
    printf("without traversal: updated pose x %f\n", a_segmentState2[1].X.p[0]);
    printf("without traversal: updated pose y %f\n", a_segmentState2[1].X.p[1]);
    printf("without traversal: updated pose z %f\n", a_segmentState2[1].X.p[2]);
    printf("without traversal: updated twist x %f\n", a_segmentState2[1].Xdot.vel[0]);
    printf("without traversal: updated twist y %f\n", a_segmentState2[1].Xdot.vel[1]);
    printf("without traversal: updated twist rot-z %f\n", a_segmentState2[1].Xdot.rot[2]);

    CPPUNIT_ASSERT(a_segmentState1[1] == a_segmentState2[1]);

}

void TraverseOperationTest::testFailedTraverseOperation()
{
    CPPUNIT_ASSERT(false);
}

