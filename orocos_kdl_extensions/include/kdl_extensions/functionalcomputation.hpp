/* 
 * File:   functionalcomputation.hpp
 * Author: azamat
 *
 * Created on August 7, 2012, 6:28 PM
 */
#include <kdl/tree.hpp>

#ifndef FUNCTIONALCOMPUTATION_HPP
#define	FUNCTIONALCOMPUTATION_HPP
//primary template for an iteration over a node(T1)
template <typename T1, typename T2, typename T3, typename OP1, typename OP2>
class iterateOver_t
{
public:
    iterateOver_t();
    T3 operator()(T1,T2,T3,OP1, OP2);
    T3 operator()(T1,T2,T3,OP1);
private:
    T3 a_internalState;

};


template <typename T2, typename T3, typename OP1, typename OP2>
class iterateOver_t<KDL::SegmentMap::const_iterator, T2, T3, OP1, OP2>
{
public:
    iterateOver_t();
    T3 operator()(KDL::SegmentMap::const_iterator,T2,T3,OP1, OP2);
    T3 operator()(KDL::SegmentMap::const_iterator,T2,T3,OP1);
private:
    T3 a_internalState;

};


template <typename T1>
class transform
{
public:
    T1 a_internalState;

};

#include "../../src/functionalcomputation.inl"
#endif	/* FUNCTIONALCOMPUTATION_HPP */

