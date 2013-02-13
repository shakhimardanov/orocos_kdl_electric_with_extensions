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

void AccTwistOperationTest::tearDown()
{
}

void AccTwistOperationTest::testTransformAccTwist()
{
    kdle::SegmentState a_segmentState1;
    KDL::SegmentMap::const_iterator segmentId = testTree.getSegment("TestSegment");
    kdle::transform<kdle::tree_iterator, kdle::accTwist> a_operation;

    std::cout << std::endl;
    printf("initial acctwist x %f\n",a_segmentState.Xdotdot.vel[0]);
    printf("initial acctwist y %f\n",a_segmentState.Xdotdot.vel[1]);
    printf("initial acctwist z %f\n",a_segmentState.Xdotdot.vel[2]);
    
    a_segmentState1 = a_operation(segmentId, a_jointState, a_segmentState);

    printf("updated acctwist x %f\n",a_segmentState1.Xdotdot.vel[0]);
    printf("updated acctwist y %f\n",a_segmentState1.Xdotdot.vel[1]);
    printf("updated acctwist z %f\n",a_segmentState1.Xdotdot.vel[2]);

    CPPUNIT_ASSERT(a_segmentState == a_segmentState1);
}

void AccTwistOperationTest::testFailedTransformAccTwist()
{
    kdle::SegmentState a_segmentState1;
    KDL::SegmentMap::const_iterator segmentId = testTree.getSegment("TestSegment");
    kdle::transform<kdle::tree_iterator, kdle::accTwist> a_operation;

    a_segmentState.Xdotdot.vel[2] = -9.8;

    std::cout << std::endl;
    printf("initial acctwist x %f\n",a_segmentState.Xdotdot.vel[0]);
    printf("initial acctwist y %f\n",a_segmentState.Xdotdot.vel[1]);
    printf("initial acctwist z %f\n",a_segmentState.Xdotdot.vel[2]);

    a_segmentState1 = a_operation(segmentId, a_jointState, a_segmentState);

    printf("updated acctwist x %f\n",a_segmentState1.Xdotdot.vel[0]);
    printf("updated acctwist y %f\n",a_segmentState1.Xdotdot.vel[1]);
    printf("updated acctwist z %f\n",a_segmentState1.Xdotdot.vel[2]);
    
    CPPUNIT_ASSERT(a_segmentState != a_segmentState1);
//    CPPUNIT_ASSERT(a_segmentState == a_segmentState1);
}

