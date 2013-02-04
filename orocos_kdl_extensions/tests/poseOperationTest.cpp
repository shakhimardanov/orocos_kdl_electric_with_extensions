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
    
    KDL::Joint testJoint = KDL::Joint("TestJoint", KDL::Joint::RotZ, 1, 0, 0.01);
    KDL::Frame testFrame(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.4, 0.0));
    KDL::Segment testSegment = KDL::Segment("TestSegment", testJoint, testFrame);
    KDL::RotationalInertia testRotInerSeg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0); //around symmetry axis of rotation
    double pointMass = 0.25; //in kg
    KDL::RigidBodyInertia testInerSegment(pointMass, KDL::Vector(0.0, 0.4, 0.0), testRotInerSeg);
    testSegment.setInertia(testInerSegment);
    
    testTree.addSegment(testSegment,"root");

    a_segmentState = kdle::SegmentState();
    a_jointState = kdle::JointState();

}

void PoseOperationTest::tearDown()
{

}

void PoseOperationTest::testTransformPose()
{
    
    kdle::SegmentState a_segmentState1;
    
    KDL::SegmentMap::const_iterator segmentId = testTree.getSegment("TestSegment");
    kdle::transform<kdle::tree_iterator, kdle::pose> a_operation;

    std::cout << std::endl;
    printf("initial pose x %f\n",a_segmentState.X.p[0]);
    printf("initial pose y %f\n",a_segmentState.X.p[1]);
    printf("initial pose z %f\n",a_segmentState.X.p[2]);

    a_segmentState1 = a_operation(segmentId, a_jointState, a_segmentState);
    
    printf("updated pose x %f\n",a_segmentState1.X.p[0]);
    printf("updated pose y %f\n",a_segmentState1.X.p[1]);
    printf("updated pose z %f\n",a_segmentState1.X.p[2]);
    
    CPPUNIT_ASSERT(a_segmentState != a_segmentState1);

}

void PoseOperationTest::testFailedTransformPose()
{
    
    kdle::SegmentState a_segmentState1;
    
    KDL::SegmentMap::const_iterator segmentId = testTree.getSegment("TestSegment");
    kdle::transform<kdle::tree_iterator, kdle::pose> a_operation;

    a_jointState.q = 0.5;
    std::cout << std::endl;
    printf("initial pose x %f\n",a_segmentState.X.p[0]);
    printf("initial pose y %f\n",a_segmentState.X.p[1]);
    printf("initial pose z %f\n",a_segmentState.X.p[2]);

    a_segmentState1 = a_operation(segmentId, a_jointState, a_segmentState);

    printf("updated pose x %f\n",a_segmentState1.X.p[0]);
    printf("updated pose y %f\n",a_segmentState1.X.p[1]);
    printf("updated pose z %f\n",a_segmentState1.X.p[2]);
    
    CPPUNIT_ASSERT(a_segmentState == a_segmentState1);

}

