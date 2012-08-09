/* 
 * File:   functionalcomputation.hpp
 * Author: azamat
 *
 * Created on August 7, 2012, 6:28 PM
 */
#include <kdl/tree.hpp>
#include <kdl_extensions/treeid_vereshchagin_composable.hpp>

#ifndef FUNCTIONALCOMPUTATION_HPP
#define	FUNCTIONALCOMPUTATION_HPP
//primary template for an iteration over a node(T1)
template <typename T_Operation1, typename T_Operation2, typename T_IterationElement = KDL::SegmentMap::const_iterator,
          typename T_ComputationalState1 = KDL::JointState, typename T_ComputationalState2 = KDL::SegmentState>
class iterateOver_t
{
public:
    iterateOver_t();
    T_ComputationalState2 operator()(T_IterationElement,T_ComputationalState1,T_ComputationalState2 ,T_Operation1, T_Operation2);
    T_ComputationalState2 operator()(T_IterationElement,T_ComputationalState1,T_ComputationalState2 ,T_Operation1);
private:
    T_ComputationalState2 a_internalState;

};




template <typename T_TransformationQuantity> //could Pose, Twist, Wrench
class transform
{
public:
    T_TransformationQuantity a_internalState;

};

#include "../../src/functionalcomputation.inl"
#endif	/* FUNCTIONALCOMPUTATION_HPP */

