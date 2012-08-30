/* 
 * File:   functionalcomputation.hpp
 * Author: azamat
 *
 * Created on August 7, 2012, 6:28 PM
 */

#ifndef FUNCTIONALCOMPUTATION_HPP
#define	FUNCTIONALCOMPUTATION_HPP

#include <kdl/tree.hpp>
#include <kdl_extensions/treeid_vereshchagin_composable.hpp>
#include <iterator>

namespace kdl_extensions
{

template <typename OperationT, int N>
class Parameter;

#define OperParamT(N)                                           \
        template<typename OperationT>                           \
        class Parameter<OperationT,N>{                          \
            public:                                             \
                typedef typename OperationT::Param##N##T Type;  \
        }

OperParamT(1);
OperParamT(2);
OperParamT(3);
OperParamT(4);
OperParamT(5);
#undef OperParamT

//operation traits new version

template <typename OperationT>
class operation_traits
{
public:
    typedef typename OperationT::NumberOfParams number_of_params;
    typedef typename OperationT::ReturnType return_type;
    typedef typename Parameter<OperationT, 1 > ::Type Param1T;
    typedef typename Parameter<OperationT, 2 > ::Type Param2T;
    typedef typename Parameter<OperationT, 3 > ::Type Param3T;
    typedef typename Parameter<OperationT, 4 > ::Type Param4T;
    typedef typename Parameter<OperationT, 5 > ::Type Param5T;
};

//operation tags

class poseOperationTag
{
};

class twistOperationTag : public poseOperationTag
{
};

class accelerationTwistOperationTag : public twistOperationTag
{
};

class wrenchOperationTag : public accelerationTwistOperationTag
{
};

typedef poseOperationTag pose;
typedef twistOperationTag twist;
typedef accelerationTwistOperationTag accTwist;
typedef wrenchOperationTag wrench;

template<typename Iterator, typename OperationTagT>
class transform;

template<typename Iterator>
class transform<Iterator, pose>
{
public:

    enum
    {
        NumberOfParams = 3
    };
    typedef KDL::SegmentState ReturnType;
    typedef Iterator Param1T;
    typedef KDL::JointState Param2T;
    typedef KDL::SegmentState Param3T;
};

template<>
class transform<KDL::SegmentMap::const_iterator, pose>
{
public:

    enum
    {
        NumberOfParams = 3
    };
    typedef KDL::SegmentState ReturnType;
    typedef KDL::SegmentMap::const_iterator Param1T;
    typedef KDL::JointState Param2T;
    typedef KDL::SegmentState Param3T;

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
private:
    ReturnType a_segmentState;

};

template<typename Iterator>
class transform<Iterator, twist>
{
public:

    enum
    {
        NumberOfParams = 3
    };
    typedef KDL::SegmentState ReturnType;
    typedef Iterator Param1T;
    typedef KDL::JointState Param2T;
    typedef KDL::SegmentState Param3T;
};

template<>
class transform<KDL::SegmentMap::const_iterator, twist>
{
public:

    enum
    {
        NumberOfParams = 3
    };
    typedef KDL::SegmentState ReturnType;
    typedef KDL::SegmentMap::const_iterator Param1T;
    typedef KDL::JointState Param2T;
    typedef KDL::SegmentState Param3T;

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
private:
    ReturnType a_segmentState;
};

template<typename Iterator>
class transform<Iterator, accTwist>
{
public:

    enum
    {
        NumberOfParams = 3
    };
    typedef KDL::SegmentState ReturnType;
    typedef Iterator Param1T;
    typedef KDL::JointState Param2T;
    typedef KDL::SegmentState Param3T;

};

template<>
class transform<KDL::SegmentMap::const_iterator, accTwist>
{
public:

    enum
    {
        NumberOfParams = 3
    };
    typedef KDL::SegmentState ReturnType;
    typedef KDL::SegmentMap::const_iterator Param1T;
    typedef KDL::JointState Param2T;
    typedef KDL::SegmentState Param3T;

    inline ReturnType operator()(Param1T segmentId, Param2T p_jointState, Param3T p_segmentState)
    {
        a_segmentState = p_segmentState;
        a_segmentState.Xdotdot = p_segmentState.X.Inverse(p_segmentState.Xdotdot) + p_segmentState.Z * p_jointState.qdotdot + p_segmentState.Xdot * p_segmentState.Vj;
        return a_segmentState;
    };
private:
    ReturnType a_segmentState;

};


//--------------------------------------------//
//for empty base class optimization

template <typename OperationT, int N>
class OperationTDerived : public OperationT
{
public:

    OperationTDerived(OperationT& a_op) : OperationT(a_op)
    {
    };

    OperationTDerived(OperationT const& a_op) : OperationT(a_op)
    {
    };
};

//composition template

template <typename OperationT1, typename OperationT2> //could be transform, project category of operations
class Compose : private OperationTDerived<OperationT1, 1 >, private OperationTDerived<OperationT2, 2 >
{
public:

    enum
    {
        NumberOfParams = operation_traits<OperationT2>::number_of_params
    };
    typedef typename operation_traits<OperationT1>::return_type ReturnType;
    typedef typename operation_traits<OperationT2>::Param1T Param1T;
    typedef typename operation_traits<OperationT2>::Param2T Param2T;
    typedef typename operation_traits<OperationT2>::Param3T Param3T;
    //ReturnType result1;

    Compose(OperationT1 a_p1, OperationT2 a_p2) : OperationTDerived<OperationT1, 1 > (a_p1), OperationTDerived<OperationT2, 2 > (a_p2)
    {
    };

    inline ReturnType operator()(Param1T a_segmentId, Param2T a_jointstate, Param3T a_linkstate)
    {

        //result1 = BaseMem<OP2, 2 >::operator()(a_segmentId, a_jointstate, a_linkstate); //why does this work?
        //return BaseMem<OP1,1>::operator()(a_segmentId, a_jointstate, result1);
        //and this one does not. IS there sth wrong with temporaries.
        return OperationTDerived<OperationT1, 1 > ::operator()(a_segmentId, a_jointstate, OperationTDerived<OperationT2, 2 > ::operator()(a_segmentId, a_jointstate, a_linkstate));

    }

};

//convinience function for composition functor

template <typename OP1, typename OP2>
inline Compose<OP1, OP2> compose(OP1 a_p1, OP2 a_p2)
{
    return Compose<OP1, OP2 > (a_p1, a_p2);
};









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



//traversal policies
//template <typename Iterator>
//class DFSPolicy;

template <typename Topology>
class DFSPolicy
{
public:

    DFSPolicy()
    {
    };

    ~DFSPolicy()
    {
    };

    template <typename OP>
    inline static bool walk(Topology a_topology, std::vector<typename OP::Param2T>& a_jointStateVectorIn, std::vector<typename OP::Param3T>& a_linkStateVectorIn,
                            std::vector<typename OP::Param3T>& a_linkStateVectorOut, OP a_op)
    {
        return true;
    };

};

template<>
class DFSPolicy<KDL::Chain>
{
public:

    DFSPolicy()
    {
    };

    ~DFSPolicy()
    {
    };

    template <typename OP>
    inline static bool walk(KDL::Chain a_topology, std::vector<typename OP::Param2T>& a_jointStateVectorIn, std::vector<typename OP::Param3T>& a_linkStateVectorIn,
                            std::vector<typename OP::Param3T>& a_linkStateVectorOut, OP a_op)
    {
        return true;
    };

};

template<>
class DFSPolicy<KDL::Tree>
{
public:

    DFSPolicy()
    {
    };

    ~DFSPolicy()
    {
    };

    template <typename OP>
    inline static bool walk(KDL::Tree a_topology, std::vector<typename OP::Param2T>& a_jointStateVectorIn, std::vector<typename OP::Param3T>& a_linkStateVectorIn,
                            std::vector<typename OP::Param3T>& a_linkStateVectorOut, OP a_op)
    {
        return true;
    };

};

//template <typename T>
//class BFSPolicy;


//traversal/schedule function
//there is an association between computationtable and topology
template<typename Topology, template <typename Topology > class TraversalPolicy, typename ComputationTable>
class IterateOverTree;


//this is for the homogeneous case, the same computation is applied on all topology elements
//template<typename Topology, template<typename > class TraversalPolicy, typename OP>

template<typename Topology, template <typename Topology > class TraversalPolicy, typename OP>
class IterateOverTree
{
public:
    typedef typename OP::ReturnType ReturnType;
    typedef typename OP::Param1T Param1T;
    typedef typename OP::Param2T Param2T;
    typedef typename OP::Param3T Param3T;

    IterateOverTree()
    {
    };

    ~IterateOverTree()
    {
    };

    inline bool operator()(Topology a_topology, std::vector<Param2T>& a_jointStateVectorIn, std::vector<Param3T>& a_linkStateVectorIn, std::vector<Param3T>& a_linkStateVectorOut, OP a_op)
    {
        return TraversalPolicy<Topology>::walk(a_topology, a_jointStateVectorIn, a_linkStateVectorIn, a_linkStateVectorOut, a_op);
    };

};

//convinience function
/*template <template <typename > class Policy, typename Topology, typename OP>
inline bool traverse(Topology a_graph, OP a_op, std::vector<typename OP::Param2T>& a_p1, std::vector<typename OP::Param3T>& a_p2, std::vector<typename OP::Param3T>& a_p3)
{
    Policy<Topology>::walk(a_graph, a_op, a_p1,a_p2,a_p3);
   
   return true;
};



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
 */

};
#include "../../src/functionalcomputation.inl"
#endif	/* FUNCTIONALCOMPUTATION_HPP */

