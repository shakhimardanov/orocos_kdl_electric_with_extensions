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


