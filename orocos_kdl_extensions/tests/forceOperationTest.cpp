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
}

void ForceOperationTest::tearDown()
{
}

void ForceOperationTest::testMethod()
{
    KDL::Segment testSegment("TestSegment");
    KDL::Tree testTree("TestTreeRoot");
    testTree.addSegment(testSegment,"TestTreeRoot");

    kdle::SegmentState a_segmentState, a_segmentState1;
    kdle::JointState a_jointState;
    KDL::SegmentMap::const_iterator segmentId = testTree.getRootSegment();
    kdle::balance<kdle::tree_iterator, kdle::force> a_operation;

    a_segmentState1 = a_operation(segmentId, a_jointState, a_segmentState);
    CPPUNIT_ASSERT(a_segmentState == a_segmentState1);
}

void ForceOperationTest::testFailedMethod()
{
    CPPUNIT_ASSERT(false);
}

