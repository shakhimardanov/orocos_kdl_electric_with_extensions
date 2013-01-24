/*
 * File:   PoseOperationTest.hpp
 * Author: azamat
 *
 * Created on Jan 24, 2013, 2:05:59 PM
 */

#ifndef POSEOPERATIONTEST_HPP
#define	POSEOPERATIONTEST_HPP

#include <cppunit/extensions/HelperMacros.h>
#include <kdl_extensions/functionalcomputation_kdltypes.hpp>
#include <kdl_extensions/treeid_vereshchagin_composable.hpp>

class PoseOperationTest : public CPPUNIT_NS::TestFixture {
    CPPUNIT_TEST_SUITE(PoseOperationTest);

    CPPUNIT_TEST(testTransformPose);
    CPPUNIT_TEST(testFailedTransformPose);

    CPPUNIT_TEST_SUITE_END();

public:
    PoseOperationTest();
    virtual ~PoseOperationTest();
    void setUp();
    void tearDown();

private:
    KDL::SegmentState *a_segmentstate;
    KDL::JointState *a_jointState;
    KDL::SegmentMap::const_iterator segmentId;
    kdle::transform<kdle::tree_iterator, kdle::pose> *a_operation;
    
    void testTransformPose();
    void testFailedTransformPose();
};

#endif	/* POSEOPERATIONTEST_HPP */

