/*
 * File:   AccTwistOperationTest.hpp
 * Author: azamat
 *
 * Created on Jan 31, 2013, 1:21:58 PM
 */

#ifndef ACCTWISTOPERATIONTEST_HPP
#define	ACCTWISTOPERATIONTEST_HPP

#include <cppunit/extensions/HelperMacros.h>
#include <kdl_extensions/functionalcomputation_kdltypes.hpp>

class AccTwistOperationTest : public CPPUNIT_NS::TestFixture {
    CPPUNIT_TEST_SUITE(AccTwistOperationTest);

    CPPUNIT_TEST(testMethod);
    CPPUNIT_TEST(testFailedMethod);

    CPPUNIT_TEST_SUITE_END();

public:
    AccTwistOperationTest();
    virtual ~AccTwistOperationTest();
    void setUp();
    void tearDown();

private:
    void testMethod();
    void testFailedMethod();
};

#endif	/* ACCTWISTOPERATIONTEST_HPP */

