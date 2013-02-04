/*
 * File:   WrenchOperationTest.cpp
 * Author: azamat
 *
 * Created on Jan 31, 2013, 2:23:08 PM
 */

#include "forceOperationTest.hpp"


CPPUNIT_TEST_SUITE_REGISTRATION(ForceOperationTest);

ForceOperationTest::ForceOperationTest()
{
}

ForceOperationTest::~ForceOperationTest()
{
}

void ForceOperationTest::setUp()
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

void ForceOperationTest::tearDown()
{
}

void ForceOperationTest::testMethod()
{
    kdle::SegmentState a_segmentState1;
    KDL::SegmentMap::const_iterator segmentId = testTree.getSegment("TestSegment");
    kdle::balance<kdle::tree_iterator, kdle::force> a_operation;

    std::cout << std::endl;
    printf("initial wrench x %f\n",a_segmentState.F.force[0]);
    printf("initial wrench y %f\n",a_segmentState.F.force[1]);
    printf("initial wrench z %f\n",a_segmentState.F.force[2]);

    a_segmentState1 = a_operation(segmentId, a_jointState, a_segmentState);

    printf("updated wrench x %f\n",a_segmentState.F.force[0]);
    printf("updated wrench y %f\n",a_segmentState.F.force[1]);
    printf("updated wrench z %f\n",a_segmentState.F.force[2]);

    CPPUNIT_ASSERT(a_segmentState == a_segmentState1);
}

void ForceOperationTest::testFailedMethod()
{
    kdle::SegmentState a_segmentState1;
    KDL::SegmentMap::const_iterator segmentId = testTree.getSegment("TestSegment");
    kdle::balance<kdle::tree_iterator, kdle::force> a_operation;

    std::cout << std::endl;
    printf("initial wrench x %f\n",a_segmentState.F.force[0]);
    printf("initial wrench y %f\n",a_segmentState.F.force[1]);
    printf("initial wrench z %f\n",a_segmentState.F.force[2]);

    a_segmentState1 = a_operation(segmentId, a_jointState, a_segmentState);

    printf("updated wrench x %f\n",a_segmentState.F.force[0]);
    printf("updated wrench y %f\n",a_segmentState.F.force[1]);
    printf("updated wrench z %f\n",a_segmentState.F.force[2]);

    CPPUNIT_ASSERT(a_segmentState != a_segmentState1);
}

