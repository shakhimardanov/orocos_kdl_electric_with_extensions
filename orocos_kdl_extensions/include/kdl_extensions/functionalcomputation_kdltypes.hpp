/* 
 * File:   functionalcomputation_kdltypes.hpp
 * Author: azamat
 *
 * Created on September 7, 2012, 3:27 PM
 */

#ifndef FUNCTIONALCOMPUTATION_KDLTYPES_HPP
#define	FUNCTIONALCOMPUTATION_KDLTYPES_HPP

#include <kdl/tree.hpp>
#include <kdl_extensions/treeid_vereshchagin_composable.hpp>
#include <kdl_extensions/functionalcomputation.hpp>
#include <kdl_extensions/computationalstate_kdltypes.hpp>

// TODO: fix all operations to take references to constant (constant input state)

namespace kdl_extensions
{
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

class inertiaOperationTag
{
};

typedef poseOperationTag pose;
typedef twistOperationTag twist;
typedef accelerationTwistOperationTag accTwist;
typedef wrenchOperationTag wrench;
typedef inertiaOperationTag inertia;

typedef std::map<std::string, KDL::TreeElement >::const_iterator tree_iterator;
typedef std::vector<KDL::Segment>::const_iterator chain_iterator;

template<typename Iterator, typename OperationTagT>
class transform;

template<>
class transform<tree_iterator, pose>
{
public:

    enum
    {
        NumberOfParams = 3
    };
    typedef KDL::SegmentState ReturnType;
    typedef tree_iterator Param1T;
    typedef KDL::JointState Param2T;
    typedef KDL::SegmentState Param3T;

    inline ReturnType operator()(Param1T segmentId, Param2T p_jointState, Param3T p_segmentState)
    {
        //check for joint type None should be tree serialization function.
        //a_segmentState.X =  a_p3.X * segmentId->second.segment.pose(p_jointState.q); //in base coordinates
        a_segmentState.Xdot = p_segmentState.Xdot;
        a_segmentState.Xdotdot = p_segmentState.Xdotdot;
        a_segmentState.F = p_segmentState.F;
        a_segmentState.X = segmentId->second.segment.pose(p_jointState.q);
        a_segmentState.jointIndex = p_jointState.jointIndex;
        a_segmentState.jointName = p_jointState.jointName;
        a_segmentState.segmentName = segmentId->first;
        std::cout << "Inside pose operation Transform value" << a_segmentState.X << std::endl;
        //std::cout << "Inside pose operation Twist value" << a_segmentState.Xdot << std::endl;
        //std::cout << "Inside pose operation AccTwist value" << a_segmentState.Xdotdot << std::endl;
        //std::cout << "Inside pose operation Wrench value" << a_segmentState.F << std::endl<< std::endl;
        return a_segmentState;

    };
private:
    ReturnType a_segmentState;

};

template<>
class transform<chain_iterator, pose>
{
public:

    enum
    {
        NumberOfParams = 3
    };
    typedef KDL::SegmentState ReturnType;
    typedef chain_iterator Param1T;
    typedef KDL::JointState Param2T;
    typedef KDL::SegmentState Param3T;

    inline ReturnType operator()(Param1T segmentId, Param2T p_jointState, Param3T p_segmentState)
    {
        //check for joint type None should be tree serialization function.
        //a_segmentState.X =  a_p3.X * segmentId->second.segment.pose(p_jointState.q); //in base coordinates
        a_segmentState.Xdot = p_segmentState.Xdot;
        a_segmentState.Xdotdot = p_segmentState.Xdotdot;
        a_segmentState.X = segmentId->pose(p_jointState.q);
        a_segmentState.jointIndex = p_jointState.jointIndex;
        a_segmentState.jointName = p_jointState.jointName;
        a_segmentState.segmentName = segmentId->getName();

        return a_segmentState;

    };
private:
    ReturnType a_segmentState;

};

template<>
class transform<tree_iterator, twist>
{
public:

    enum
    {
        NumberOfParams = 3
    };
    typedef KDL::SegmentState ReturnType;
    typedef tree_iterator Param1T;
    typedef KDL::JointState Param2T;
    typedef KDL::SegmentState Param3T;

    inline ReturnType operator()(Param1T segmentId, Param2T p_jointState, Param3T p_segmentState)
    {
        a_segmentState.X = p_segmentState.X;
        a_segmentState.Xdotdot = p_segmentState.Xdotdot;
        a_segmentState.F = p_segmentState.F;
        a_segmentState.Z = a_segmentState.X.M.Inverse(segmentId->second.segment.twist(p_jointState.q, 1.0));
        //a_segmentState.Z = a_jointUnitTwist;
        a_segmentState.Vj = a_segmentState.X.M.Inverse(segmentId->second.segment.twist(p_jointState.q, p_jointState.qdot));
        
        //do we check here for the index of a joint (whether the joint is first/root in the chain)
        //if so, somehow an information about whether a segment is root or not should be sent here
        a_segmentState.Xdot = a_segmentState.X.Inverse(p_segmentState.Xdot) + a_segmentState.Vj;
        //std::cout << "Inside twist operation Transform value" << a_segmentState.X << std::endl;
        std::cout << "Inside twist operation Twist value" << a_segmentState.Xdot << std::endl;
       // std::cout << "Inside twist operation AccTwist value" << a_segmentState.Xdotdot << std::endl;
       // std::cout << "Inside twist operation Wrench value" << a_segmentState.F << std::endl<< std::endl;
        return a_segmentState;
    };
private:
    ReturnType a_segmentState;
};

template<>
class transform<chain_iterator, twist>
{
public:

    enum
    {
        NumberOfParams = 3
    };
    typedef KDL::SegmentState ReturnType;
    typedef chain_iterator Param1T;
    typedef KDL::JointState Param2T;
    typedef KDL::SegmentState Param3T;

    inline ReturnType operator()(Param1T segmentId, Param2T p_jointState, Param3T p_segmentState)
    {
        a_segmentState.X = p_segmentState.X;
        a_segmentState.Xdotdot = p_segmentState.Xdotdot;
        a_segmentState.Z = a_segmentState.X.M.Inverse(segmentId->twist(p_jointState.q, 1.0));
        //a_segmentState.Z = a_jointUnitTwist;
        a_segmentState.Vj = a_segmentState.X.M.Inverse(segmentId->twist(p_jointState.q, p_jointState.qdot));
        //a_segmentState.Vj = a_jointTwistVelocity;
        //do we check here for the index of a joint (whether the joint is first in the chain)
        a_segmentState.Xdot = a_segmentState.X.Inverse(p_segmentState.Xdot) + a_segmentState.Vj;

        return a_segmentState;
    };
private:
    ReturnType a_segmentState;
};

template<>
class transform<tree_iterator, accTwist>
{
public:

    enum
    {
        NumberOfParams = 3
    };
    typedef KDL::SegmentState ReturnType;
    typedef tree_iterator Param1T;
    typedef KDL::JointState Param2T;
    typedef KDL::SegmentState Param3T;

    inline ReturnType operator()(Param1T segmentId, Param2T p_jointState, Param3T p_segmentState)
    {
        a_segmentState = p_segmentState;
        a_segmentState.Xdotdot = p_segmentState.X.Inverse(p_segmentState.Xdotdot) + p_segmentState.Z * p_jointState.qdotdot + p_segmentState.Xdot * p_segmentState.Vj;
        //std::cout << "Inside acctwist operation Transform value" << a_segmentState.X << std::endl;
        //std::cout << "Inside acctwist operation Twist value" << a_segmentState.Xdot << std::endl;
        std::cout << "Inside acctwist operation AccTwist value" << a_segmentState.Xdotdot << std::endl;
        //std::cout << "Inside acctwist operation Wrench value" << a_segmentState.F << std::endl<< std::endl;
        return a_segmentState;
    };
private:
    ReturnType a_segmentState;

};

template<>
class transform<chain_iterator, accTwist>
{
public:

    enum
    {
        NumberOfParams = 3
    };
    typedef KDL::SegmentState ReturnType;
    typedef chain_iterator Param1T;
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

template<typename Iterator, typename OperationTagT>
class project;

template<>
class project<chain_iterator, wrench>
{
public:

    enum
    {
        NumberOfParams = 3
    };
    typedef KDL::SegmentState ReturnType;
    typedef chain_iterator Param1T;
    typedef KDL::JointState Param2T;
    typedef KDL::SegmentState Param3T;

    inline ReturnType operator()(Param1T segmentId, Param2T p_jointState, Param3T p_segmentState)
    {
        a_segmentState = p_segmentState;
        a_segmentState.F = segmentId->getInertia() * a_segmentState.Xdotdot + a_segmentState.Xdot * (segmentId->getInertia() * a_segmentState.Xdot) - a_segmentState.Fext;
        return a_segmentState;
    };
private:
    ReturnType a_segmentState;

};

template<>
class project<tree_iterator, wrench>
{
public:

    enum
    {
        NumberOfParams = 3
    };
    typedef KDL::SegmentState ReturnType;
    typedef tree_iterator Param1T;
    typedef KDL::JointState Param2T;
    typedef KDL::SegmentState Param3T;

    inline ReturnType operator()(Param1T segmentId, Param2T p_jointState, Param3T p_segmentState)
    {
        a_segmentState = p_segmentState;
        a_segmentState.F = segmentId->second.segment.getInertia() * a_segmentState.Xdotdot + a_segmentState.Xdot * (segmentId->second.segment.getInertia() * a_segmentState.Xdot) - a_segmentState.Fext;
        /*
        std::cout << "Inside wrench operation Transform value " << a_segmentState.X << std::endl;
        std::cout << "Inside wrench operation Twist value " << a_segmentState.Xdot << std::endl;
        std::cout << "Inside wrench operation AccTwist value " << a_segmentState.Xdotdot << std::endl;
        */
        std::cout << "Inside wrench operation Wrench value " << a_segmentState.F << std::endl<< std::endl;
        return a_segmentState;
    };
private:
    ReturnType a_segmentState;

};

template<>
class project<chain_iterator, inertia>
{
public:

    enum
    {
        NumberOfParams = 3
    };
    typedef KDL::RigidBodyInertia ReturnType;
    typedef chain_iterator Param1T;
    typedef KDL::JointState Param2T;
    typedef KDL::SegmentState Param3T;

    inline ReturnType operator()(Param1T segmentId, Param2T p_jointState, Param3T p_segmentState)
    {
        return segmentId->getInertia();
    };

};

template<>
class project<tree_iterator, inertia>
{
public:

    enum
    {
        NumberOfParams = 3
    };
    typedef KDL::RigidBodyInertia ReturnType;
    typedef tree_iterator Param1T;
    typedef KDL::JointState Param2T;
    typedef KDL::SegmentState Param3T;

    inline ReturnType operator()(Param1T segmentId, Param2T p_jointState, Param3T p_segmentState)
    {
        return segmentId->second.segment.getInertia();
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

    //TODO: fix this parameters(state vectors are passed by value). As in the case of operations it should be ref to const.
    template <typename OP>
    inline static bool walk(KDL::Tree a_topology, std::vector<typename OP::Param2T> a_jointStateVectorIn, std::vector<typename OP::Param3T> a_linkStateVectorIn,
                            std::vector<typename OP::Param3T> a_linkStateVectorOut, OP a_op)
    {
        //just a simple test, should implement DFS algorithm
        //we should also provide a global computational state to the operations
        //how do we refer to the previous element in the link state vector
        //maybe composed operation itself should not be called  directly but through another helper operation which also takes
        //vector length info since we always need previous link state info
        
        for (KDL::SegmentMap::const_iterator iter = a_topology.getSegments().begin(); iter != a_topology.getSegments().end(); ++iter)
        {
            const KDL::TreeElement parentElement = iter->second;
            std::cout << "Parent element name in current iteration " << parentElement.segment.getName() << std::endl;
            std::cout << "Current/parent joint index and value " << parentElement.q_nr << " " << a_jointStateVectorIn[parentElement.q_nr].q << std::endl;
            for (std::vector<KDL::SegmentMap::const_iterator>::const_iterator childIter = iter->second.children.begin(); childIter != iter->second.children.end(); childIter++)
            {

                std::cout << "Child element name in current iteration " << (*childIter)->second.segment.getName() << std::endl;
                std::cout << "Current/child joint index and value " << (*childIter)->second.q_nr << " " << a_jointStateVectorIn[(*childIter)->second.q_nr].q << std::endl;
                a_linkStateVectorIn[(*childIter)->second.q_nr] = a_op(*childIter, a_jointStateVectorIn[(*childIter)->second.q_nr], a_linkStateVectorIn[parentElement.q_nr]);
                a_linkStateVectorOut[parentElement.q_nr] = a_linkStateVectorIn[(*childIter)->second.q_nr];
            }
        };
        return true;
    };

};

template<>
class BFSPolicy<KDL::Tree>
{
public:

    BFSPolicy()
    {
    };

    ~BFSPolicy()
    {
    };

    template <typename OP>
    inline static bool walk(KDL::Tree a_topology, std::vector<typename OP::Param2T> a_jointStateVectorIn, std::vector<typename OP::Param3T> a_linkStateVectorIn,
                            std::vector<typename OP::Param3T> a_linkStateVectorOut, OP a_op)
    {
        //just a simple test, will implement DFS algorithm
        for (KDL::SegmentMap::const_iterator iter = a_topology.getSegments().begin(); iter != a_topology.getSegments().end(); ++iter)
        {
            a_op(iter, a_jointStateVectorIn[0], a_linkStateVectorIn[0]);
        };
        return true;
    };

};

};

#include "../../src/functionalcomputation_kdltypes.inl"
#endif	/* FUNCTIONALCOMPUTATION_KDLTYPES_HPP */

