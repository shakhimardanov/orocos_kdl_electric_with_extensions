/*
 * File:   TwistOperationTest.hpp
 * Author: azamat
 *
 * Created on Jan 31, 2013, 1:16:00 PM
 */

#ifndef TWISTOPERATIONTEST_HPP
#define	TWISTOPERATIONTEST_HPP

#include <cppunit/extensions/HelperMacros.h>
#include <kdl_extensions/functionalcomputation_kdltypes.hpp>

class TwistOperationTest : public CPPUNIT_NS::TestFixture {
    CPPUNIT_TEST_SUITE(TwistOperationTest);

    CPPUNIT_TEST(testMethod);
    CPPUNIT_TEST(testFailedMethod);

    CPPUNIT_TEST_SUITE_END();

public:
    TwistOperationTest();
    virtual ~TwistOperationTest();
    void setUp();
    void tearDown();

private:
    void testMethod();
    void testFailedMethod();
};

#endif	/* TWISTOPERATIONTEST_HPP */

