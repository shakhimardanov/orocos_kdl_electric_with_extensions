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

    KDL::Joint testJoint = KDL::Joint("TestJoint", KDL::Joint::RotZ, 1, 0, 0.01);
    KDL::Frame testFrame(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, -0.4, 0.0));
    KDL::Segment testSegment = KDL::Segment("TestSegment", testJoint, testFrame);
    KDL::RotationalInertia testRotInerSeg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0); //around symmetry axis of rotation
    double pointMass = 0.25; //in kg
    KDL::RigidBodyInertia testInerSegment(pointMass, KDL::Vector(0.0, -0.4, 0.0), testRotInerSeg);
    testSegment.setInertia(testInerSegment);

    testTree.addSegment(testSegment,"root");

    a_segmentState = kdle::SegmentState();
    a_jointState = kdle::JointState();

}

void TwistOperationTest::tearDown()
{
}

void TwistOperationTest::testTransformTwist()
{
    
    kdle::SegmentState a_segmentState1;
    KDL::SegmentMap::const_iterator segmentId = testTree.getSegment("TestSegment");
    kdle::transform<kdle::kdl_tree_iterator, kdle::twist> a_operation;

    a_segmentState.Xdot.vel[0] = 2.0;
    a_jointState.qdot = 0.2;
    std::cout << std::endl;
    printf("initial twist x %f\n",a_segmentState.Xdot.vel[0]);
    printf("initial twist y %f\n",a_segmentState.Xdot.vel[1]);
    printf("initial twist rot-z %f\n",a_segmentState.Xdot.rot[2]);

    a_segmentState1 = a_operation(segmentId, a_jointState, a_segmentState);

    printf("updated twist x %f\n",a_segmentState1.Xdot.vel[0]);
    printf("updated twist y %f\n",a_segmentState1.Xdot.vel[1]);
    printf("updated twist rot-z %f\n",a_segmentState1.Xdot.rot[2]);

    CPPUNIT_ASSERT(a_segmentState != a_segmentState1);
}

void TwistOperationTest::testFailedTransformTwist()
{
    kdle::SegmentState a_segmentState1;
    KDL::SegmentMap::const_iterator segmentId = testTree.getSegment("TestSegment");
    kdle::transform<kdle::kdl_tree_iterator, kdle::twist> a_operation;


    std::cout << std::endl;
    printf("initial twist x %f\n",a_segmentState.Xdot.vel[0]);
    printf("initial twist y %f\n",a_segmentState.Xdot.vel[1]);
    printf("initial twist rot-z %f\n",a_segmentState.Xdot.rot[2]);

    a_segmentState1 = a_operation(segmentId, a_jointState, a_segmentState);

    printf("updated twist x %f\n",a_segmentState1.Xdot.vel[0]);
    printf("updated twist y %f\n",a_segmentState1.Xdot.vel[1]);
    printf("updated twist rot-z %f\n",a_segmentState1.Xdot.rot[2]);

    CPPUNIT_ASSERT(a_segmentState != a_segmentState1);
//    CPPUNIT_ASSERT(a_segmentState == a_segmentState1);
}

