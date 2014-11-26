/*
 * File:   TraverseOperationTest.h
 * Author: azamat
 *
 * Created on Feb 12, 2013, 3:05:31 PM
 */

#ifndef TRAVERSEOPERATIONTEST_H
#define	TRAVERSEOPERATIONTEST_H

#include <cppunit/extensions/HelperMacros.h>
#include <kdl_extensions/functionalcomputation_kdl.hpp>

class TraverseOperationTest : public CPPUNIT_NS::TestFixture
{
    CPPUNIT_TEST_SUITE(TraverseOperationTest);

    CPPUNIT_TEST(testTraverseOperation);
    CPPUNIT_TEST(testFailedTraverseOperation);

    CPPUNIT_TEST_SUITE_END();

public:
    TraverseOperationTest();
    virtual ~TraverseOperationTest();
    void setUp();
    void tearDown();

private:
    KDL::Tree testTree;
    std::vector<kdle::SegmentState> a_segmentState;
    std::vector<kdle::JointState> a_jointState;
    kdle::transform<kdle::kdl_tree_iterator, kdle::pose> a_operation1;
    kdle::transform<kdle::kdl_tree_iterator, kdle::twist> a_operation2;
    
    kdle::DFSPolicy<KDL::Tree, kdle::outward> a_policy1;

    void testTraverseOperation();
    void testFailedTraverseOperation();
};

#endif	/* TRAVERSEOPERATIONTEST_H */

