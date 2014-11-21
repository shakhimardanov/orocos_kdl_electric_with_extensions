/*
 * File:   WrenchOperationTest.hpp
 * Author: azamat
 *
 * Created on Jan 31, 2013, 2:23:08 PM
 */

#ifndef WRENCHOPERATIONTEST_HPP
#define	WRENCHOPERATIONTEST_HPP



#include <cppunit/extensions/HelperMacros.h>
#include <kdl_extensions/functionalcomputation_kdl.hpp>

class ForceOperationTest : public CPPUNIT_NS::TestFixture {
    CPPUNIT_TEST_SUITE(ForceOperationTest);

    CPPUNIT_TEST(testBalanceWrench);
    CPPUNIT_TEST(testFailedBalanceWrench);

    CPPUNIT_TEST_SUITE_END();

public:
    ForceOperationTest();
    virtual ~ForceOperationTest();
    void setUp();
    void tearDown();

private:
    KDL::Tree testTree;
    kdle::SegmentState a_segmentState;
    kdle::JointState a_jointState;

    void testBalanceWrench();
    void testFailedBalanceWrench();
};

#endif	/* WRENCHOPERATIONTEST_HPP */

