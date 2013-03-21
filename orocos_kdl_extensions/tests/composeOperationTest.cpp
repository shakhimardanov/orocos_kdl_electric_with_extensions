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
    kdle::JointState a_jointState1;

    a_jointState.q = KDL::PI / 3.0;
    a_jointState.qdot = 0.2;

    printf("initial pose x %f\n", a_segmentState.X.p[0]);
    printf("initial pose y %f\n", a_segmentState.X.p[1]);
    printf("initial pose z %f\n", a_segmentState.X.p[2]);
    printf("initial twist x %f\n", a_segmentState.Xdot.vel[0]);
    printf("initial twist y %f\n", a_segmentState.Xdot.vel[1]);
    printf("initial twist z %f\n", a_segmentState.Xdot.vel[2]);
    //    update a state by performing two consecutive operations on it
    //    in this case it is first pose transform and then twist transform
    //3 arguments
    a_segmentState1 = a_operation1(testTree.getSegment("TestSegment"), a_jointState, a_segmentState);
    a_segmentState2 = a_operation2(testTree.getSegment("TestSegment"), a_jointState, a_segmentState1);

  
    //5 arguments
    a_segmentState1 = a_operation1(testTree.getSegment("TestSegment"), a_jointState, a_segmentState, a_jointState, a_segmentState1);
    a_segmentState2 = a_operation2(testTree.getSegment("TestSegment"), a_jointState, a_segmentState, a_jointState, a_segmentState1);

    printf("without composition: updated pose x %f\n", a_segmentState2.X.p[0]);
    printf("without composition: updated pose y %f\n", a_segmentState2.X.p[1]);
    printf("without composition: updated pose z %f\n", a_segmentState2.X.p[2]);
    printf("without composition: updated twist x %f\n", a_segmentState2.Xdot.vel[0]);
    printf("without composition: updated twist y %f\n", a_segmentState2.Xdot.vel[1]);
    printf("without composition: updated twist rot-z %f\n", a_segmentState2.Xdot.rot[2]);

    //    update a state applying composed operations on it
        //    in this case pose and twist transform operations are functionally composed
    a_segmentState1 = kdle::compose(a_operation2, a_operation1) (testTree.getSegment("TestSegment"), a_jointState, a_segmentState);

//    4 arguments
//    a_segmentState2 = kdle::compose(a_operation2, a_operation1) (testTree.getSegment("TestSegment"), a_jointState, a_segmentState, a_jointState1, a_segmentState1);

    printf("composition: updated pose x %f\n", a_segmentState1.X.p[0]);
    printf("composition: updated pose y %f\n", a_segmentState1.X.p[1]);
    printf("composition: updated pose z %f\n", a_segmentState1.X.p[2]);
    printf("composition: updated twist x %f\n", a_segmentState1.Xdot.vel[0]);
    printf("composition: updated twist y %f\n", a_segmentState1.Xdot.vel[1]);
    printf("composition: updated twist rot-z %f\n", a_segmentState1.Xdot.rot[2]);

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
    
//    4 argument operation test
//    a_segmentState1 = kdle::compose(kdle::compose(a_operation4, a_operation3), kdle::compose(a_operation2, a_operation1)) (testTree.getSegment("TestSegment"), a_jointState, a_segmentState, a_jointState, a_segmentState2);


    CPPUNIT_ASSERT(a_segmentState2 != a_segmentState1);
}

