/****************************************************************************** 
*           This file is part of the Geometric harmonization project          *
*                                                                             *
*                            (C) 2014 Azamat Shakhimardanov                   *
*                               Herman Bruyninckx                             *
*                        azamat.shakhimardanov@mech.kuleuven.be               *                              
*                    Department of Mechanical Engineering,                    *
*                   Katholieke Universiteit Leuven, Belgium.                  *
*                                                                             *
*       You may redistribute this software and/or modify it under either the  *
*       terms of the GNU Lesser General Public License version 2.1 (LGPLv2.1  *
*       <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>) or (at your *
*       discretion) of the Modified BSD License:                              *
*       Redistribution and use in source and binary forms, with or without    *
*       modification, are permitted provided that the following conditions    *
*       are met:                                                              *
*       1. Redistributions of source code must retain the above copyright     *
*       notice, this list of conditions and the following disclaimer.         *
*       2. Redistributions in binary form must reproduce the above copyright  *
*       notice, this list of conditions and the following disclaimer in the   *
*       documentation and/or other materials provided with the distribution.  *
*       3. The name of the author may not be used to endorse or promote       *
*       products derived from this software without specific prior written    *
*       permission.                                                           *
*       THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR  *
*       IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED        *
*       WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE    *
*       ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,*
*       INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES    *
*       (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS       *
*       OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) *
*       HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,   *
*       STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING *
*       IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE    *
*       POSSIBILITY OF SUCH DAMAGE.                                           *
*                                                                             *
*******************************************************************************/

#ifndef FUNCTIONALCOMPUTATION_HPP
#define	FUNCTIONALCOMPUTATION_HPP

#include <iterator>
#include <kdl_extensions/functionalcomputation_util.hpp>


namespace kdle
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
        std::cout << "Composite operation is created " << std::endl;
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
        std::cout << "2 argument composition call" << std::endl;
        return OperationTDerived<OperationT1, 1 > ::operator()(a_param1, OperationTDerived<OperationT2, 2 > ::operator()(a_param1, a_param2));
    };

    //overloaded for three params
    //compose(f,g)(x,y,t) will give the following result
    //f(x,y,z) where z = g(x,y,t)
    inline ReturnType operator()(typename ParameterTypeQualifier<Param1T>::RefToConstT a_param1,
                                 typename ParameterTypeQualifier<Param2T>::RefToConstT a_param2,
                                 typename ParameterTypeQualifier<Param3T>::RefToConstT a_param3)
    {
        std::cout << "3 argument composition call" << std::endl;
        return OperationTDerived<OperationT1, 1 > ::operator()(a_param1, a_param2, OperationTDerived<OperationT2, 2 > ::operator()(a_param1, a_param2, a_param3));
    };

     //overloaded for four params
    //compose(f,g)(x,y,z,t) will give the following result
    //f(x,y,z,w) where w = g(x,y,z,t)
    inline ReturnType operator()(typename ParameterTypeQualifier<Param1T>::RefToConstT a_param1,
                                 typename ParameterTypeQualifier<Param2T>::RefToConstT a_param2,
                                 typename ParameterTypeQualifier<Param3T>::RefToConstT a_param3,
                                 typename ParameterTypeQualifier<Param4T>::RefToConstT a_param4)
    {
        std::cout << "4 argument composition call" << std::endl;
        return OperationTDerived<OperationT1, 1 > ::operator()(a_param1, a_param2, a_param3, OperationTDerived<OperationT2, 2 > ::operator()(a_param1, a_param2, a_param3, a_param4));
    };

    inline ReturnType operator()(typename ParameterTypeQualifier<Param1T>::RefToConstT a_param1,
                                 typename ParameterTypeQualifier<Param2T>::RefToConstT a_param2,
                                 typename ParameterTypeQualifier<Param3T>::RefToConstT a_param3,
                                 typename ParameterTypeQualifier<Param4T>::RefToArgT a_param4,
                                 typename ParameterTypeQualifier<Param5T>::RefToConstT a_param5)
    {
        std::cout << "5 argument composition call" << std::endl;
        return OperationTDerived<OperationT1, 1 > ::operator()(a_param1, a_param2, a_param3, a_param4, OperationTDerived<OperationT2, 2 > ::operator()(a_param1, a_param2, a_param3, a_param4, a_param5));
    };

    //can add further overloads when needed

    
};

//convenience  function for composition functor

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
class BFSPolicy;

//this is a test DFS version which allows to define a direction of traversal
//the direction of a sweep is defined with respect to a root or leaves of the tree structure
//thus any sweep is outgoing when it starts as the root and any sweep is ingoing when it starts at the leaves.
enum Direction{outward=0, inward=1};

template <typename Topology, Direction sweepDirection=outward>
class DFSPolicy;

/*
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
        std::cout << "Traverse operation is created " << std::endl;
    };

    ~IterateOver()
    {

    };
    
    // TODO: need to make type of container a template parameter
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

//convenience function

template <typename Topology, typename OP, template <typename > class Policy>
inline IterateOver<Topology, OP, Policy> traverseGraph(Topology a_graph, OP a_op, Policy<Topology> a_policy)
{
  
    return IterateOver<Topology, OP, Policy > (a_graph, a_op, a_policy);
};

*/

//this is a version of traversal which works with DFS_ver2
template<typename Topology, typename OperationT, Direction sweepDirection, 
         template <typename Topology, Direction sweepDirection=outward> class TraversalPolicy>
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
    IterateOver()
    {
        std::cout << "Traverse operation is created " << std::endl;
    };
    //Constructor
    IterateOver(typename ParameterTypeQualifier<Topology>::RefToConstT a_topol,
                typename ParameterTypeQualifier<OperationT>::RefToConstT a_oper,
                Direction sweepDir,
                typename ParameterTypeQualifier<TraversalPolicy<Topology, sweepDirection> >::RefToConstT policy) :
                a_graph(a_topol), a_op(a_oper), a_policy(policy)
    {
        std::cout << "Traverse operation is created " << std::endl;
    };

    ~IterateOver()
    {

    };


    inline bool operator()(typename ParameterTypeQualifier<std::vector<Param2T> >::RefToConstT a_param1,
                           typename ParameterTypeQualifier<std::vector<Param3T> >::RefToArgT a_param2,
                           typename ParameterTypeQualifier<std::vector<Param3T> >::RefToArgT a_param3)
    {
        std::cout << "3 argument traverse call" << std::endl;
        return a_policy.walk(a_graph, a_param1, a_param2, a_param3, a_op);
    };


    inline bool operator()(typename ParameterTypeQualifier<std::vector<Param2T> >::RefToConstT a_param1,
                           typename ParameterTypeQualifier<std::vector<Param2T> >::RefToArgT   a_param2,
                           typename ParameterTypeQualifier<std::vector<Param3T> >::RefToConstT a_param3,
                           typename ParameterTypeQualifier<std::vector<Param3T> >::RefToArgT a_param4)
    {
        std::cout << "4 argument traverse call" << std::endl;
        return a_policy.walk(a_graph, a_param1, a_param2, a_param3, a_param4, a_op);
    };
private:
    Topology a_graph;
    OperationT a_op;
    TraversalPolicy<Topology, sweepDirection> a_policy;

};


template <typename Topology, typename OP, Direction sweepDir, template <typename, Direction> class Policy>
inline IterateOver<Topology, OP, sweepDir , Policy> traverseGraph(Topology a_graph, OP a_op, Policy<Topology, sweepDir> a_policy)
{
   
    return IterateOver<Topology, OP, sweepDir, Policy > (a_graph, a_op, sweepDir, a_policy);
};

};
#endif	/* FUNCTIONALCOMPUTATION_HPP */


