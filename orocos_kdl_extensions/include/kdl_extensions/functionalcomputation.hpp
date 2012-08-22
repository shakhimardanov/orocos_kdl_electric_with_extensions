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

namespace kdl_extensions
{

//operation arguments and return related traits
//primary template

template<typename RT, typename Arg1T, typename Arg2T, typename Arg3T>
class OperationTraits
{
public:
    typedef RT ReturnType;
    typedef Arg1T Param1T;
    typedef Arg2T Param2T;
    typedef Arg3T Param3T;
};

//specialization for KDL types
//Trait for Pose transform Operation
class _Pose : public OperationTraits<KDL::SegmentState, KDL::SegmentMap::const_iterator, KDL::JointState, KDL::SegmentState>
{

};

class _Twist : public OperationTraits<KDL::SegmentState, KDL::SegmentMap::const_iterator, KDL::JointState, KDL::SegmentState>
{
};

class _AccTwist : public OperationTraits<KDL::SegmentState, KDL::SegmentMap::const_iterator, KDL::JointState, KDL::SegmentState>
{
};

class _Wrench : public OperationTraits<KDL::SegmentState, KDL::SegmentMap::const_iterator, KDL::JointState, KDL::SegmentState>
{
};

//now use the trait above in the computational operations
//Transform functor primary template

template<typename T>
class Transform
{
public:

    Transform()
    {
    };

    ~Transform()
    {
    };

    Transform(Transform& copy)
    {
    };
};

//Tranform functor specialized with Pose trait

template<>
class Transform<_Pose>
{
public:
    typedef _Pose::ReturnType ReturnType;
    typedef _Pose::Param1T Param1T;
    typedef _Pose::Param2T Param2T;
    typedef _Pose::Param3T Param3T;
protected:
    ReturnType a_segmentState;
public:

    enum
    {
        NumberOfParams = 3
    };

    inline ReturnType operator()(Param1T segmentId, Param2T p_jointState, Param3T p_segmentState)
    {
        //check for joint type None should be tree serialization function.
        //a_segmentState.X =  a_p3.X * segmentId->second.segment.pose(p_jointState.q); //in base coordinates
        a_segmentState.Xdot = p_segmentState.Xdot;
        a_segmentState.Xdotdot = p_segmentState.Xdotdot;
        a_segmentState.X = segmentId->second.segment.pose(p_jointState.q);
        a_segmentState.jointIndex = p_jointState.jointIndex;
        a_segmentState.jointName = p_jointState.jointName;
        a_segmentState.segmentName = segmentId->first;
        std::cout << "Inside transformPose 0" << a_segmentState.X << std::endl;
        return a_segmentState;

    };

};

template<>
class Transform<_Twist>
{
public:
    typedef _Twist::ReturnType ReturnType;
    typedef _Twist::Param1T Param1T;
    typedef _Twist::Param2T Param2T;
    typedef _Twist::Param3T Param3T;
protected:
    ReturnType a_segmentState;
public:

    enum
    {
        NumberOfParams = 3
    };

    inline ReturnType operator()(Param1T segmentId, Param2T p_jointState, Param3T p_segmentState)
    {
        a_segmentState.X = p_segmentState.X;
        a_segmentState.Xdotdot = p_segmentState.Xdotdot;
        a_segmentState.Z = a_segmentState.X.M.Inverse(segmentId->second.segment.twist(p_jointState.q, 1.0));
        //a_segmentState.Z = a_jointUnitTwist;
        a_segmentState.Vj = a_segmentState.X.M.Inverse(segmentId->second.segment.twist(p_jointState.q, p_jointState.qdot));
        //a_segmentState.Vj = a_jointTwistVelocity;

        //do we check here for the index of a joint (whether the joint is first in the chain)
        a_segmentState.Xdot = a_segmentState.X.Inverse(p_segmentState.Xdot) + a_segmentState.Vj;

        return a_segmentState;
    };
};

template<>
class Transform<_AccTwist>
{
public:
    typedef _AccTwist::ReturnType ReturnType;
    typedef _AccTwist::Param1T Param1T;
    typedef _AccTwist::Param2T Param2T;
    typedef _AccTwist::Param3T Param3T;
protected:
    ReturnType a_segmentState;
public:

    enum
    {
        NumberOfParams = 3
    };

    inline ReturnType operator()(Param1T segmentId, Param2T p_jointState, Param3T p_segmentState)
    {
        a_segmentState = p_segmentState;
        a_segmentState.Xdotdot = p_segmentState.X.Inverse(p_segmentState.Xdotdot) + p_segmentState.Z * p_jointState.qdotdot + p_segmentState.Xdot * p_segmentState.Vj;
        return a_segmentState;
    };
};

//--------------------------------------------//
//for empty base class optimization

template <typename C, int N>
class BaseMem : public C
{
public:

    BaseMem(C& c) : C(c)
    {
    };

    BaseMem(C const& c) : C(c)
    {
    };
};

//composition template

template <typename OP1, typename OP2> //could Pose, Twist, Wrench
class Compose : private BaseMem<OP1, 1 >, private BaseMem<OP2, 2 >
{
public:

    enum
    {
        NumberOfParams = OP2::NumberOfParams
    };
    typedef typename OP1::ReturnType ReturnType;
    typedef typename OP2::Param1T Param1T;
    typedef typename OP2::Param2T Param2T;
    typedef typename OP2::Param3T Param3T;

    Compose(OP1 a_p1, OP2 a_p2) : BaseMem<OP1, 1 > (a_p1), BaseMem<OP2, 2 > (a_p2)
    {
    };

    inline ReturnType operator()(Param1T a_segmentId, Param2T a_jointstate, Param3T a_linkstate)
    {

        return BaseMem<OP1,1>::operator()(a_segmentId, a_jointstate, BaseMem<OP2, 2 >::operator()(a_segmentId, a_jointstate, a_linkstate));
        //BaseMem<OP1, 1 > ::operator()(a_segmentId, a_jointstate, BaseMem<OP2, 2 > ::operator()(a_segmentId, a_jointstate, a_linkstate));
    }

};

//convinience function for composition functor

template <typename OP1, typename OP2>
inline Compose<OP1, OP2> compose(OP1 a_p1, OP2 a_p2)
{
    return Compose<OP1, OP2 > (a_p1, a_p2);
};



//primary template for an iteration over a node(T1)
//template <typename IterationElementT, typename JointStateT, typename SegmentStateT, typename OP>
//OP::RT iterate
//template <typename IterationElement, typename StateTraits<KDL::JointState>::StateTraitType, typename StateTraits<KDL::SegmentState>::StateTraitType, typename OP>

template <typename T_Operation1, typename T_Operation2, typename T_IterationElement = KDL::SegmentMap::const_iterator,
        typename T_ComputationalState1 = KDL::JointState, typename T_ComputationalState2 = KDL::SegmentState>
        class iterateOver_t
{
public:
    iterateOver_t();
    T_ComputationalState2 operator()(T_IterationElement, T_ComputationalState1, T_ComputationalState2, T_Operation1, T_Operation2);
    T_ComputationalState2 operator()(T_IterationElement, T_ComputationalState1, T_ComputationalState2, T_Operation1);
private:
    T_ComputationalState2 a_internalState;

};

};
#include "../../src/functionalcomputation.inl"
#endif	/* FUNCTIONALCOMPUTATION_HPP */

