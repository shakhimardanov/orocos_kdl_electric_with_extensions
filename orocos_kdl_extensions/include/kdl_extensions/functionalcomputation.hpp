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

//traits for types used with computational operations
template<typename T>
class StateTraits;

template<>
class StateTraits<KDL::JointState>
{
public:
    typedef KDL::JointState StateTraitType;

};

template<>
class StateTraits<KDL::SegmentState>
{
public:
    typedef KDL::SegmentState StateTraitType;

};



//--------------------------------------------//
//for empty base call optimization
template <typename C, int N>
class BaseMem: public C
{
public:
    BaseMem(C& c): C(c){};
    BaseMem(C const& c): C(c){};
};

//composition template
template <typename OP1, typename OP2> //could Pose, Twist, Wrench
class Composer :private BaseMem<OP1, 1>,
                private BaseMem<OP2, 2>
{
public:
    enum {NumberOfParams = OP2::NumberOfParams};
    typedef typename OP1::ReturnType ReturnType;
    typedef typename OP2::Param1T Param1T;

    Composer(OP1 a_p1, OP2 a_p2): BaseMem<OP1,1>(a_p1), BaseMem<OP2,2>(a_p2){ };
    
    ReturnType operator()(KDL::SegmentMap::const_iterator a_segmentId, KDL::JointState& a_jointstate, KDL::SegmentState& a_linkstate)
    {

        return BaseMem<OP1,1>::operator()(a_segmentId, a_jointstate, BaseMem<OP2,2>(a_segmentId, a_jointstate, a_linkstate));
    }

};

//convinience function for composition functor
template <typename OP1, typename OP2>
inline Composer<OP1, OP2> composer(OP1 a_p1, OP2 a_p2)
{
    return Composer<OP1, OP2>(a_p1, a_p2);
}



//primary template for an iteration over a node(T1)
//template <typename IterationElement, typename OP>
///OP::RT iterate


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

#include "../../src/functionalcomputation.inl"
#endif	/* FUNCTIONALCOMPUTATION_HPP */

