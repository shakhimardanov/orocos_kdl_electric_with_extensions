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

    Composite(typename ParameterTypeQualifier<OperationT1>::RefToConstT a_p1,
              typename ParameterTypeQualifier<OperationT2>::RefToConstT a_p2) :
              OperationTDerived<OperationT1, 1 > (a_p1), OperationTDerived<OperationT2, 2 > (a_p2)
    {
    };

    //overloaded for one param
    //compose(f,g)(t) will give the following result
    //f(x), where x = g(t)
    inline ReturnType operator()(typename ParameterTypeQualifier<Param1T>::RefToConstT a_param1)
    {
        return OperationTDerived<OperationT1, 1 > ::operator()(OperationTDerived<OperationT2, 2 > ::operator()(a_param1));
    };
    //overloaded for two params
    //compose(f,g)(x,y) will give the following result
    //f(x,z), where z = g(x,y)
    inline ReturnType operator()(typename ParameterTypeQualifier<Param1T>::RefToConstT a_param1,
                                 typename ParameterTypeQualifier<Param2T>::RefToConstT a_param2)
    {
        return OperationTDerived<OperationT1, 1 > ::operator()(a_param1, OperationTDerived<OperationT2, 2 > ::operator()(a_param1, a_param2));
    };

    //overloaded for three params
    //compose(f,g)(x,y,t) will give the following result
    //f(x,y,z) where z = g(x,y,t)
    inline ReturnType operator()(typename ParameterTypeQualifier<Param1T>::RefToConstT a_param1,
                                 typename ParameterTypeQualifier<Param2T>::RefToConstT a_param2,
                                 typename ParameterTypeQualifier<Param3T>::RefToConstT a_param3)
    {
        return OperationTDerived<OperationT1, 1 > ::operator()(a_param1, a_param2, OperationTDerived<OperationT2, 2 > ::operator()(a_param1, a_param2, a_param3));
    };
    //can add further overloads when needed

};

//convinience function for composition functor

template <typename OperationT1, typename OperationT2>
inline Composite<OperationT1, OperationT2> compose(OperationT1 a_p1, OperationT2 a_p2)
{
    return Composite<OperationT1, OperationT2 > (a_p1, a_p2);
};

//TODO: the composition above is defined for computational operations
//one should define a composition for tree walking operations.
//Composition of forward and reverse walks



//traversal policies
//primary templates
template <typename Topology>
class DFSPolicy;

template <typename Topology>
class BFSPolicy;

//this is a test DFS version which allows to define a direction of traversal
enum Direction{outward=0, inward=1};

template <typename Topology, Direction sweepDirection=outward>
class DFSPolicy_ver2;





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

    //TODO: check parameter qualifiers
    //Constructor
    IterateOver(typename ParameterTypeQualifier<Topology>::RefToConstT a_topol,
                typename ParameterTypeQualifier<OperationT>::RefToConstT a_oper,
                typename ParameterTypeQualifier<TraversalPolicy<Topology> >::RefToConstT policy) :
                a_graph(a_topol), a_op(a_oper), a_policy(policy)
    {

    };

    ~IterateOver()
    {

    };

    // TODO: make this function call general/independent of any domain. Currently parameter names imply that it is for kinematic chains
    // also need to make type of container a template parameter
    // e.g template <typename ParamT, typename Allocator = allocator<ParamT> > class Container = std::vector
    // this traversal can only be used with operations which take three arguments
    // need to introduce the other versions too.

    inline bool operator()(typename ParameterTypeQualifier<std::vector<Param2T> >::RefToConstT a_param1,
                           typename ParameterTypeQualifier<std::vector<Param3T> >::RefToArgT a_param2,
                           typename ParameterTypeQualifier<std::vector<Param3T> >::RefToArgT a_param3)
    // 2nd and 3rd params have the same type because computations are returned by a value, whereas traversal are return by a reference param.
    //Think of a better solution
    {
        return a_policy.forwardwalk(a_graph, a_param1, a_param2, a_param3, a_op);
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
    //Policy<Topology>::forwardwalk(a_graph, a_op, a_p1,a_p2,a_p3);

    return IterateOver<Topology, OP, Policy > (a_graph, a_op, a_policy);
};





//this is a version of traversal which works with DFS_ver2
template<typename Topology, typename OperationT, Direction sweepDirection, 
         template <typename Topology, Direction sweepDirection=outward> class TraversalPolicy>
class IterateOver_ver2
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

    //TODO: check parameter qualifiers
    //Constructor
    IterateOver_ver2(typename ParameterTypeQualifier<Topology>::RefToConstT a_topol,
                typename ParameterTypeQualifier<OperationT>::RefToConstT a_oper,
                typename ParameterTypeQualifier<TraversalPolicy<Topology, sweepDirection> >::RefToConstT policy) :
                a_graph(a_topol), a_op(a_oper), a_policy(policy)
    {

    };

    ~IterateOver_ver2()
    {

    };


    inline bool operator()(typename ParameterTypeQualifier<std::vector<Param2T> >::RefToConstT a_param1,
                           typename ParameterTypeQualifier<std::vector<Param3T> >::RefToArgT a_param2,
                           typename ParameterTypeQualifier<std::vector<Param3T> >::RefToArgT a_param3)
    {
        return a_policy.walk(a_graph, a_param1, a_param2, a_param3, a_op);
    };
private:
    Topology a_graph;
    OperationT a_op;
    TraversalPolicy<Topology, sweepDirection> a_policy;

};


};

#include "../../src/functionalcomputation.inl"
#endif	/* FUNCTIONALCOMPUTATION_HPP */


