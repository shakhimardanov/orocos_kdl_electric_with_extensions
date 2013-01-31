/*
 * File:   WrenchOperationTest.hpp
 * Author: azamat
 *
 * Created on Jan 31, 2013, 2:23:08 PM
 */

#ifndef WRENCHOPERATIONTEST_HPP
#define	WRENCHOPERATIONTEST_HPP

#include <cppunit/extensions/HelperMacros.h>
#include <kdl_extensions/functionalcomputation_kdltypes.hpp>

class ForceOperationTest : public CPPUNIT_NS::TestFixture {
    CPPUNIT_TEST_SUITE(ForceOperationTest);

    CPPUNIT_TEST(testMethod);
    CPPUNIT_TEST(testFailedMethod);

    CPPUNIT_TEST_SUITE_END();

public:
    ForceOperationTest();
    virtual ~ForceOperationTest();
    void setUp();
    void tearDown();

private:
    void testMethod();
    void testFailedMethod();
};

#endif	/* WRENCHOPERATIONTEST_HPP */

