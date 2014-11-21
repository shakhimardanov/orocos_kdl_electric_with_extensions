/*
 * File:   ComposeOperationTest.hpp
 * Author: azamat
 *
 * Created on Feb 12, 2013, 1:00:41 PM
 */

#ifndef COMPOSEOPERATIONTEST_HPP
#define	COMPOSEOPERATIONTEST_HPP

#include <cppunit/extensions/HelperMacros.h>
#include <kdl_extensions/functionalcomputation_kdl.hpp>

class ComposeOperationTest : public CPPUNIT_NS::TestFixture
{
    CPPUNIT_TEST_SUITE(ComposeOperationTest);

    CPPUNIT_TEST(testComposition);
    CPPUNIT_TEST(testFailedComposition);

    CPPUNIT_TEST_SUITE_END();

public:
    ComposeOperationTest();
    virtual ~ComposeOperationTest();
    void setUp();
    void tearDown();

private:

    KDL::Tree testTree;
    kdle::SegmentState a_segmentState;
    kdle::JointState a_jointState;
    kdle::transform<kdle::tree_iterator, kdle::pose> a_operation1;
    kdle::transform<kdle::tree_iterator, kdle::twist> a_operation2;
    kdle::transform<kdle::tree_iterator, kdle::accTwist> a_operation3;
    kdle::balance<kdle::tree_iterator, kdle::force> a_operation4;

    void testComposition();
    void testFailedComposition();
};

#endif	/* COMPOSEOPERATIONTEST_HPP */

