/* 
 * File:   functionalcomputation.hpp
 * Author: azamat
 *
 * Created on August 7, 2012, 6:28 PM
 */

#ifndef FUNCTIONALCOMPUTATION_HPP
#define	FUNCTIONALCOMPUTATION_HPP

#include <iterator>
#include <kdl_extensions/functionalcomputation_util.hpp>


namespace kdl_extensions
{

//operation traits new version
//should make this recursive like in typelist

//--------------------------------------------//
//for empty base class optimization

template <typename OperationT, int N>
class OperationTDerived : public OperationT
{
public:

    OperationTDerived(typename ParameterTypeQualifier<OperationT>::RefToT a_op) : OperationT(a_op)
    {
    };

    OperationTDerived(typename ParameterTypeQualifier<OperationT>::RefToConstT a_op) : OperationT(a_op)
    {
    };
};

//composition template
//later should enable any function with any number of parameters

template <typename OperationT1, typename OperationT2> //could be transform, project category of operations
class Composite : private OperationTDerived<OperationT1, 1 >, private OperationTDerived<OperationT2, 2 >
{
public:

    enum
    {
        NumberOfParams = OperationT2::NumberOfParams
    };

    typedef typename OperationT1::ReturnType ReturnType;

    typedef typename OperationTParameterType<OperationT2, 1 > ::Type Param1T;
    typedef typename OperationTParameterType<OperationT2, 2 > ::Type Param2T;
    typedef typename OperationTParameterType<OperationT2, 3 > ::Type Param3T;
    typedef typename OperationTParameterType<OperationT2, 4 > ::Type Param4T;
    typedef typename OperationTParameterType<OperationT2, 5 > ::Type Param5T;
    typedef typename OperationTParameterType<OperationT2, 6 > ::Type Param6T;
    typedef typename OperationTParameterType<OperationT2, 7 > ::Type Param7T;

    Composite(typename ParameterTypeQualifier<OperationT1>::RefToConstT a_p1, typename ParameterTypeQualifier<OperationT2>::RefToConstT a_p2) : OperationTDerived<OperationT1, 1 > (a_p1), OperationTDerived<OperationT2, 2 > (a_p2)
    {
    };

    //overloaded for one param

    inline ReturnType operator()(typename ParameterTypeQualifier<Param1T>::RefToConstT a_param1)
    {
        return OperationTDerived<OperationT1, 1 > ::operator()(OperationTDerived<OperationT2, 2 > ::operator()(a_param1));
    };
    //overloaded for two params

    inline ReturnType operator()(typename ParameterTypeQualifier<Param1T>::RefToConstT a_param1, typename ParameterTypeQualifier<Param2T>::RefToConstT a_param2)
    {
        return OperationTDerived<OperationT1, 1 > ::operator()(a_param1, OperationTDerived<OperationT2, 2 > ::operator()(a_param1, a_param2));
    };

    //overloaded for three params

    inline ReturnType operator()(typename ParameterTypeQualifier<Param1T>::RefToConstT a_segmentId, typename ParameterTypeQualifier<Param2T>::RefToConstT a_jointstate, typename ParameterTypeQualifier<Param3T>::RefToConstT a_linkstate)
    {
        return OperationTDerived<OperationT1, 1 > ::operator()(a_segmentId, a_jointstate, OperationTDerived<OperationT2, 2 > ::operator()(a_segmentId, a_jointstate, a_linkstate));
    };
    //can add further overloads when needed

};

//convinience function for composition functor

template <typename OperationT1, typename OperationT2>
inline Composite<OperationT1, OperationT2> compose(OperationT1 a_p1, OperationT2 a_p2)
{
    return Composite<OperationT1, OperationT2 > (a_p1, a_p2);
};

/*
//operation arguments and return related traits
//primary template/old version
template<typename RT, typename Arg1T, typename Arg2T, typename Arg3T>
class OperationTraits;

template<typename RT, typename Arg1T, typename Arg2T, typename Arg3T>
class OperationTraits<RT, Arg1T, Arg2T const &, Arg3T const &>
{
public:

    enum
    {
        NumberOfParameters = 3
    };
    typedef RT ReturnType;
    typedef Arg1T Param1T;
    typedef Arg2T const & Param2TRef;
    typedef Arg3T const & Param3TRef;
};

template<typename RT, typename Arg1T, typename Arg2T, typename Arg3T>
class OperationTraits<RT, Arg1T&, Arg2T&, Arg3T&>
{
public:

    enum
    {
        NumberOfParameters = 3
    };
    typedef RT ReturnType;
    typedef Arg1T& Param1TRef;
    typedef Arg2T& Param2TRef;
    typedef Arg3T& Param3TRef;
};

template<typename RT, typename Arg1T, typename Arg2T, typename Arg3T>
class OperationTraits<RT, Arg1T*, Arg2T&, Arg3T&>
{
public:

    enum
    {
        NumberOfParameters = 3
    };
    typedef RT ReturnType;
    typedef Arg1T* Param1TPoint;
    typedef Arg2T& Param2TRef;
    typedef Arg3T& Param3TRef;
};
//specialization for KDL types
//Trait for Pose transform Operation

class pose_t : public OperationTraits<KDL::SegmentState, KDL::SegmentMap::const_iterator, KDL::JointState const &, KDL::SegmentState const &>
{
};

class twist_t : public OperationTraits<KDL::SegmentState, KDL::SegmentMap::const_iterator, KDL::JointState const &, KDL::SegmentState const &>
{
};

class acctwist_t : public OperationTraits<KDL::SegmentState, KDL::SegmentMap::const_iterator, KDL::JointState const &, KDL::SegmentState const &>
{
};

class wrench_t : public OperationTraits<KDL::SegmentState, KDL::SegmentMap::const_iterator, KDL::JointState const &, KDL::SegmentState const &>
{
};

//now use the trait above in the computational operations
//Transform functor primary template

template<typename T>
class Transform;


//Tranform functor specialized with Pose trait

template<>
class Transform<pose_t>
{
public:
    typedef pose_t::ReturnType ReturnType;
    typedef pose_t::Param1T Param1T;
    typedef pose_t::Param2TRef Param2T;
    typedef pose_t::Param3TRef Param3T;
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
class Transform<twist_t>
{
public:
    typedef twist_t::ReturnType ReturnType;
    typedef twist_t::Param1T Param1T;
    typedef twist_t::Param2TRef Param2T;
    typedef twist_t::Param3TRef Param3T;
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
class Transform<acctwist_t>
{
public:
    typedef acctwist_t::ReturnType ReturnType;
    typedef acctwist_t::Param1T Param1T;
    typedef acctwist_t::Param2TRef Param2T;
    typedef acctwist_t::Param3TRef Param3T;
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

//primary template for computational function Project

template<typename T>
class Project
{
public:

    Project()
    {
    };

    Project(Project& copy)
    {
    };

    ~Project()
    {
    };
};

//Specialization for Wrench types

template<>
class Project<wrench_t>
{
public:
    typedef wrench_t::ReturnType ReturnType;
    typedef wrench_t::Param1T Param1T;
    typedef wrench_t::Param2TRef Param2T;
    typedef wrench_t::Param3TRef Param3T;

    inline ReturnType operator()(Param1T segmentId, Param2T p_jointState, Param3T p_segmentState)
    {
    };
};
 */

//traversal policies
//primary templates
template <typename Topology>
class DFSPolicy;

template <typename Topology>
class BFSPolicy;


//traversal/schedule function
//there is an association between computationtable and topology
template<typename Topology, typename ComputationTable, template <typename Topology > class TraversalPolicy = DFSPolicy >
class IterateOver;


//this is for the homogeneous case, the same computation is applied on all topology elements
//template<typename Topology, template<typename > class TraversalPolicy, typename OP>
template<typename Topology, typename OperationT, template <typename Topology > class TraversalPolicy>
class IterateOver
{
public:
    typedef typename OperationT::ReturnType ReturnType;

    typedef typename OperationTParameterType<OperationT, 1 > ::Type Param1T;
    typedef typename OperationTParameterType<OperationT, 2 > ::Type Param2T;
    typedef typename OperationTParameterType<OperationT, 3 > ::Type Param3T;
    typedef typename OperationTParameterType<OperationT, 4 > ::Type Param4T;
    typedef typename OperationTParameterType<OperationT, 5 > ::Type Param5T;
    typedef typename OperationTParameterType<OperationT, 6 > ::Type Param6T;
    typedef typename OperationTParameterType<OperationT, 7 > ::Type Param7T;

    IterateOver(typename ParameterTypeQualifier<Topology>::RefToConstT a_topol, typename ParameterTypeQualifier<OperationT>::RefToConstT a_oper, typename ParameterTypeQualifier<TraversalPolicy<Topology> >::RefToConstT policy) :
    a_graph(a_topol), a_op(a_oper), a_policy(policy)
    {
    };

    ~IterateOver()
    {
    };

    inline bool operator()(std::vector<Param2T> a_jointStateVectorIn, std::vector<Param3T> a_linkStateVectorIn, std::vector<Param3T> a_linkStateVectorOut)
    {
        return a_policy.walk(a_graph, a_jointStateVectorIn, a_linkStateVectorIn, a_linkStateVectorOut, a_op);
    };
private:
    Topology a_graph;
    OperationT a_op;
    TraversalPolicy<Topology> a_policy;

};

//convinience function
template <typename Topology, typename OP, template <typename > class Policy>
inline IterateOver<Topology, OP, Policy> traverseGraph(Topology a_graph, OP a_op, Policy<Topology> a_policy)
{
    //Policy<Topology>::walk(a_graph, a_op, a_p1,a_p2,a_p3);

    return IterateOver<Topology, OP, Policy > (a_graph, a_op, a_policy);
};


};

#include "../../src/functionalcomputation.inl"
#endif	/* FUNCTIONALCOMPUTATION_HPP */


