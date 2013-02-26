/* 
 * File:   functionalcomputation_kdltypes.hpp
 * Author: azamat
 *
 * Created on September 7, 2012, 3:27 PM
 */

#ifndef FUNCTIONALCOMPUTATION_KDLTYPES_HPP
#define	FUNCTIONALCOMPUTATION_KDLTYPES_HPP


#include <kdl/tree.hpp>
#include <kdl_extensions/computationalstate_kdltypes.hpp>
#include <kdl_extensions/functionalcomputation.hpp>


namespace kdle
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

class spatialForceOperationTag : public accelerationTwistOperationTag
{
};

class wrenchOperationTag : public spatialForceOperationTag
{
};

class inertiaOperationTag
{
};

typedef poseOperationTag pose;
typedef twistOperationTag twist;
typedef accelerationTwistOperationTag accTwist;
typedef spatialForceOperationTag force;
typedef wrenchOperationTag wrench;
typedef inertiaOperationTag inertia;

typedef std::map<std::string, KDL::TreeElement >::const_iterator tree_iterator;
typedef std::vector<KDL::Segment>::const_iterator chain_iterator;


template <typename Iterator>//, typename Operation> //accumulation operation (* or +)
class accumulate
{
public:
    enum
    {
        NumberOfParams = 3
    };
    typedef SegmentState ReturnType;
    typedef Iterator Param1T;
    typedef JointState Param2T;
    typedef SegmentState Param3T;

    //this needs to be changes, currently this is just overwritten
    accumulate(ParameterTypeQualifier<Param3T>::RefToConstT initialValue)
    {
        a_segmentState = initialValue;
    };

    inline ReturnType operator()(typename ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
                                 ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
                                 ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState1)
    {
        a_segmentState = p_segmentState1;
        a_segmentState.Xtotal = a_segmentState.Xtotal*p_segmentState1.X;
        return a_segmentState;
    };
private:
    SegmentState a_segmentState;
};

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
    typedef SegmentState ReturnType;
    typedef tree_iterator Param1T;
    typedef JointState Param2T;
    typedef SegmentState Param3T;

    inline ReturnType operator()(ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
            ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
            ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState)
    {
        //check for joint type None should be tree serialization function.
        //a_segmentState.X =  a_p3.X * segmentId->second.segment.pose(p_jointState.q); //in base coordinates
        a_segmentState.Xdot = p_segmentState.Xdot;
        a_segmentState.Xdotdot = p_segmentState.Xdotdot;
        a_segmentState.F = p_segmentState.F;
        a_segmentState.X = segmentId->second.segment.pose(p_jointState.q);
        a_segmentState.Xtotal = p_segmentState.Xtotal;
        a_segmentState.jointIndex = p_jointState.jointIndex;
        a_segmentState.jointName = p_jointState.jointName;
        a_segmentState.segmentName = segmentId->first;
#ifdef VERBOSE_CHECK
        std::cout << "Inside pose operation Transform value" << std::endl << a_segmentState.X << std::endl;
//        std::cout << "Inside pose operation Twist value" << a_segmentState.Xtotal << std::endl;
        //std::cout << "Inside pose operation AccTwist value" << a_segmentState.Xdotdot << std::endl;
        //std::cout << "Inside pose operation Wrench value" << a_segmentState.F << std::endl<< std::endl;
#endif 
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
    typedef SegmentState ReturnType;
    typedef chain_iterator Param1T;
    typedef JointState Param2T;
    typedef SegmentState Param3T;

    inline ReturnType operator()(ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
            ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
            ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState)
    {
        //check for joint type None should be tree serialization function.
        //a_segmentState.X =  a_p3.X * segmentId->second.segment.pose(p_jointState.q); //in base coordinates
        a_segmentState.Xdot = p_segmentState.Xdot;
        a_segmentState.Xdotdot = p_segmentState.Xdotdot;
        a_segmentState.X = segmentId->pose(p_jointState.q);
        a_segmentState.jointIndex = p_jointState.jointIndex;
        a_segmentState.jointName = p_jointState.jointName;
        a_segmentState.segmentName = segmentId->getName();

#ifdef VERBOSE_CHECK
        std::cout << "Inside pose operation Transform value" << std::endl << a_segmentState.X << std::endl;
#endif
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
    typedef SegmentState ReturnType;
    typedef tree_iterator Param1T;
    typedef JointState Param2T;
    typedef SegmentState Param3T;

    inline ReturnType operator()(ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
            ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
            ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState)
    {
        a_segmentState.X = p_segmentState.X;
        a_segmentState.Xdotdot = p_segmentState.Xdotdot;
        a_segmentState.F = p_segmentState.F;
        a_segmentState.Z = a_segmentState.X.M.Inverse(segmentId->second.segment.twist(p_jointState.q, 1.0));
        a_segmentState.Vj = a_segmentState.X.M.Inverse(segmentId->second.segment.twist(p_jointState.q, p_jointState.qdot));

        //do we check here for the index of a joint (whether the joint is first/root in the chain)
        //if so, somehow an information about whether a segment is root or not should be sent here
        a_segmentState.Xdot = a_segmentState.X.Inverse(p_segmentState.Xdot) + a_segmentState.Vj;
#ifdef VERBOSE_CHECK
        //std::cout << "Inside twist operation Transform value" << a_segmentState.X << std::endl;
        std::cout << "Inside twist operation Twist value" << std::endl << a_segmentState.Xdot << std::endl;
        // std::cout << "Inside twist operation AccTwist value" << a_segmentState.Xdotdot << std::endl;
        // std::cout << "Inside twist operation Wrench value" << a_segmentState.F << std::endl<< std::endl;
#endif
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
    typedef SegmentState ReturnType;
    typedef chain_iterator Param1T;
    typedef JointState Param2T;
    typedef SegmentState Param3T;

    inline ReturnType operator()(ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
            ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
            ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState)
    {
        a_segmentState.X = p_segmentState.X;
        a_segmentState.Xdotdot = p_segmentState.Xdotdot;
        a_segmentState.Z = a_segmentState.X.M.Inverse(segmentId->twist(p_jointState.q, 1.0));
        a_segmentState.Vj = a_segmentState.X.M.Inverse(segmentId->twist(p_jointState.q, p_jointState.qdot));
        //do we check here for the index of a joint (whether the joint is first in the chain)
        a_segmentState.Xdot = a_segmentState.X.Inverse(p_segmentState.Xdot) + a_segmentState.Vj;
#ifdef VERBOSE_CHECK
        std::cout << "Inside twist operation Twist value" << std::endl << a_segmentState.Xdot << std::endl;
#endif
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
    typedef SegmentState ReturnType;
    typedef tree_iterator Param1T;
    typedef JointState Param2T;
    typedef SegmentState Param3T;

    inline ReturnType operator()(ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
            ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
            ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState)
    {
        a_segmentState = p_segmentState;
        a_segmentState.Xdotdot = p_segmentState.X.Inverse(p_segmentState.Xdotdot) + p_segmentState.Z * p_jointState.qdotdot + p_segmentState.Xdot * p_segmentState.Vj;
#ifdef VERBOSE_CHECK
        //std::cout << "Inside acctwist operation Transform value" << a_segmentState.X << std::endl;
        //std::cout << "Inside acctwist operation Twist value" << a_segmentState.Xdot << std::endl;
        std::cout << "Inside acctwist operation AccTwist value" << std::endl << a_segmentState.Xdotdot << std::endl;
        //std::cout << "Inside acctwist operation Wrench value" << a_segmentState.F << std::endl<< std::endl;
#endif
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
    typedef SegmentState ReturnType;
    typedef chain_iterator Param1T;
    typedef JointState Param2T;
    typedef SegmentState Param3T;

    inline ReturnType operator()(ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
            ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
            ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState)
    {
        a_segmentState = p_segmentState;
        a_segmentState.Xdotdot = p_segmentState.X.Inverse(p_segmentState.Xdotdot) + p_segmentState.Z * p_jointState.qdotdot + p_segmentState.Xdot * p_segmentState.Vj;
#ifdef VERBOSE_CHECK
        std::cout << "Inside acctwist operation AccTwist value" << std::endl << a_segmentState.Xdotdot << std::endl;
#endif
        return a_segmentState;
    };
private:
    ReturnType a_segmentState;

};

template<typename Iterator, typename OperationTagT>
class balance;

template<>
class balance<chain_iterator, force>
{
public:

    enum
    {
        NumberOfParams = 3
    };
    typedef SegmentState ReturnType;
    typedef chain_iterator Param1T;
    typedef JointState Param2T;
    typedef SegmentState Param3T;

    inline ReturnType operator()(ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
            ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
            ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState)
    {
        a_segmentState = p_segmentState;
        a_segmentState.F = segmentId->getInertia() * a_segmentState.Xdotdot + a_segmentState.Xdot * (segmentId->getInertia() * a_segmentState.Xdot) - a_segmentState.Fext;
#ifdef VERBOSE_CHECK
        std::cout << "Inside wrench operation Wrench value " << std::endl << a_segmentState.F << std::endl << std::endl;
#endif
        return a_segmentState;
    };
private:
    ReturnType a_segmentState;

};

template<>
class balance<tree_iterator, force>
{
public:

    enum
    {
        NumberOfParams = 3
    };
    typedef SegmentState ReturnType;
    typedef tree_iterator Param1T;
    typedef JointState Param2T;
    typedef SegmentState Param3T;

    inline ReturnType operator()(ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
            ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
            ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState)
    {
        a_segmentState = p_segmentState;
        a_segmentState.F = segmentId->second.segment.getInertia() * a_segmentState.Xdotdot + a_segmentState.Xdot * (segmentId->second.segment.getInertia() * a_segmentState.Xdot) - a_segmentState.Fext;
#ifdef VERBOSE_CHECK
        //        std::cout << "Inside wrench operation Transform value " << a_segmentState.X << std::endl;
        //        std::cout << "Inside wrench operation Twist value " << a_segmentState.Xdot << std::endl;
        //        std::cout << "Inside wrench operation AccTwist value " << a_segmentState.Xdotdot << std::endl;
        std::cout << "Inside wrench operation Wrench value " << std::endl << a_segmentState.F << std::endl << std::endl;
#endif
        return a_segmentState;
    };
private:
    ReturnType a_segmentState;

};

template<typename Iterator, typename OperationTagT>
class project;

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
    typedef JointState Param2T;
    typedef SegmentState Param3T;

    inline ReturnType operator()(ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
            ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
            ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState)
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
    typedef JointState Param2T;
    typedef SegmentState Param3T;

    inline ReturnType operator()(ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
            ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
            ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState)
    {
        return segmentId->second.segment.getInertia();
    };
};

template<>
class project<tree_iterator, wrench>
{
public:

    enum
    {
        NumberOfParams = 3
    };
    typedef SegmentState ReturnType;
    typedef tree_iterator Param1T;
    typedef JointState Param2T;
    typedef SegmentState Param3T;

    inline ReturnType operator()(ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
            ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
            ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState)
    {
        //TODO: Place holder for future use
        return a_segmentState;
    };

    inline ReturnType operator()(ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
            ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
            ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState, //current's state (current's body force)
            ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState2) //current's child's state (child's total force)
    {

        a_segmentState = p_segmentState;
        JointState a_jointState = p_jointState;

#ifdef VERBOSE_CHECK
        std::cout << "Inside wrench operation Input Wrench of current element (current's body wrench) " << std::endl << p_segmentState.F << std::endl << std::endl;
        std::cout << "Inside wrench operation Input Wrench of a child (child's total wrench) " << std::endl << p_segmentState2.F << std::endl << std::endl;
        //        std::cout << "Inside wrench operation Input Wrench copy of current element "<< std::endl << a_segmentState.F << std::endl << std::endl;
#endif

        a_jointState.torque = dot(p_segmentState2.Z, p_segmentState2.F);
        a_segmentState.F = p_segmentState.F + p_segmentState2.X * p_segmentState2.F;

#ifdef VERBOSE_CHECK
        std::cout << "Inside wrench operation updated Wrench of current element " << std::endl << a_segmentState.F << std::endl << std::endl;
        std::cout << "Inside wrench operation computed Torque of current joint " << std::endl << a_jointState.torque << std::endl << std::endl;
#endif
        return a_segmentState;
    };

    inline ReturnType operator()(ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
            ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
            ParameterTypeQualifier<Param2T>::RefToArgT p_jointState2,
            ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState, //current's state (current's body force)
            ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState2) //current's child's state (child's total force)
    {

        a_segmentState = p_segmentState;
        
#ifdef VERBOSE_CHECK
        std::cout << "Inside wrench operation Input Wrench of current element (current's body wrench) " << std::endl << p_segmentState.F << std::endl << std::endl;
        std::cout << "Inside wrench operation Input Wrench of a child (child's total wrench) " << std::endl << p_segmentState2.F << std::endl << std::endl;
#endif

        p_jointState2.torque = dot(p_segmentState2.Z, p_segmentState2.F);
        a_segmentState.F = p_segmentState.F + p_segmentState2.X * p_segmentState2.F;

#ifdef VERBOSE_CHECK
        std::cout << "Inside wrench operation updated Wrench of current element " << std::endl << a_segmentState.F << std::endl << std::endl;
        std::cout << "Inside wrench operation computed Torque of current joint " << std::endl << p_jointState2.torque << std::endl << std::endl;
#endif
        return a_segmentState;
    };


private:
    ReturnType a_segmentState;

};




//TODO: Deprecate this policy in favor of DFSPolicy_ver2
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
    inline static bool forwardwalk(typename ParameterTypeQualifier<KDL::Chain>::RefToConstT a_topology,
                                   typename ParameterTypeQualifier<std::vector<typename OP::Param2T> >::RefToConstT a_jointStateVectorIn,
                                   typename ParameterTypeQualifier<std::vector<typename OP::Param3T> >::RefToArgT a_linkStateVectorIn,
                                   typename ParameterTypeQualifier<std::vector<typename OP::Param3T> >::RefToArgT a_linkStateVectorOut,
                                   OP a_op)
    {
        int jointindex = 0;
        for (chain_iterator iter = a_topology.segments.begin(); iter != a_topology.segments.end(); ++iter)
        {

            a_linkStateVectorIn[jointindex] = a_op(iter, a_jointStateVectorIn[jointindex], a_linkStateVectorIn[jointindex]);
            ++jointindex;
        };

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
    //we should also provide a global computational state to the operations
    //how do we refer to the previous element in the link state vector
    //maybe composed operation itself should not be called  directly but through another helper operation which also takes
    //vector length info since we always need previous link state info

    template <typename OP>
    inline static bool forwardwalk(typename ParameterTypeQualifier<KDL::Tree>::RefToConstT a_topology,
                                   typename ParameterTypeQualifier<std::vector<typename OP::Param2T> >::RefToConstT a_jointStateVectorIn,
                                   //typename ParameterTypeQualifier<std::vector<typename OP::Param2T> >::RefToArgT a_jointStateVectorIn, //introduce a separate mutable state representation, now is used for testing
                                   typename ParameterTypeQualifier<std::vector<typename OP::Param3T> >::RefToArgT a_linkStateVectorIn,
                                   typename ParameterTypeQualifier<std::vector<typename OP::Param3T> >::RefToArgT a_linkStateVectorOut,
                                   OP a_op)
    {

        //this is forward/outward iterative walk
        for (KDL::SegmentMap::const_iterator iter = a_topology.getSegments().begin(); iter != a_topology.getSegments().end(); ++iter)
        {
            const KDL::TreeElement parentElement = iter->second;
#ifdef VERBOSE_CHECK
            std::cout << "Parent element name in current iteration " << parentElement.segment.getName() << std::endl;
            std::cout << "Current/parent joint index and value " << parentElement.q_nr << " " << a_jointStateVectorIn[parentElement.q_nr].q << std::endl;
#endif
            a_linkStateVectorOut[parentElement.q_nr] = a_linkStateVectorIn[parentElement.q_nr];
            for (std::vector<KDL::SegmentMap::const_iterator>::const_iterator childIter = iter->second.children.begin(); childIter != iter->second.children.end(); childIter++)
            {
#ifdef VERBOSE_CHECK
                std::cout << "Child element name in current iteration " << (*childIter)->second.segment.getName() << std::endl;
                std::cout << "Current/child joint index and value " << (*childIter)->second.q_nr << " " << a_jointStateVectorIn[(*childIter)->second.q_nr].q << std::endl;
#endif

                a_linkStateVectorIn[(*childIter)->second.q_nr] = a_op(*childIter, a_jointStateVectorIn[(*childIter)->second.q_nr], a_linkStateVectorIn[parentElement.q_nr]);

            }
        }

#ifdef VERBOSE_CHECK
        std::cout << std::endl << "This is reverse iteration/inward" << std::endl;
#endif
        //this is reverse/inward iterative walk
        for (KDL::SegmentMap::const_reverse_iterator iter = a_topology.getSegments().rbegin(); iter != a_topology.getSegments().rend(); ++iter)
        {
            const KDL::TreeElement parentElement = iter->second;
#ifdef VERBOSE_CHECK

            std::cout << "Parent element name in current reverse iteration " << parentElement.segment.getName() << std::endl;
            std::cout << "Current/parent joint index and value in reverse iteration " << parentElement.q_nr << " " << a_jointStateVectorIn[parentElement.q_nr].q << std::endl << std::endl;
#endif
            //TODO: make torque accessible. In order to do this we need to introduce mutable joint computational state.
            //in total having 4 (2 immutable and mutable per link and per joint)
            //also need to put this iteration into a separate reverse walk
            for (std::vector<KDL::SegmentMap::const_iterator>::const_iterator childIter = iter->second.children.begin(); childIter != iter->second.children.end(); childIter++)
            {
                //                  torques(j--)=dot(S[i],f[i]);
                //                  f[i - 1] = f[i - 1] + X[i] * f[i];
                //the second term should be summed for all children of the parent and then added to the parent's force.
                a_linkStateVectorIn[parentElement.q_nr].F = a_linkStateVectorIn[parentElement.q_nr].F + a_linkStateVectorIn[(*childIter)->second.q_nr].X * a_linkStateVectorIn[(*childIter)->second.q_nr].F;
                double torque = dot(a_linkStateVectorIn[(*childIter)->second.q_nr].Z, a_linkStateVectorIn[(*childIter)->second.q_nr].F);



#ifdef VERBOSE_CHECK

                std::cout << "Child element name in current  reverse iteration " << (*childIter)->second.segment.getName() << std::endl;
                std::cout << "Current/child joint index and value " << (*childIter)->second.q_nr << " " << a_jointStateVectorIn[(*childIter)->second.q_nr].q << std::endl;
                std::cout << "Total spatial force on a parent " << a_linkStateVectorIn[parentElement.q_nr].F << std::endl;
                std::cout << "Total spatial force on a child " << a_linkStateVectorIn[(*childIter)->second.q_nr].F << std::endl;
                std::cout << "Torque at the curent joint " << torque << std::endl << std::endl;
#endif

            }

        }

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



//Direction here is the type name of enum

template<>
class DFSPolicy_ver2<KDL::Tree, outward>
{
public:

    DFSPolicy_ver2()
    {
    };

    ~DFSPolicy_ver2()
    {
    };

    template <typename OP>
    inline static bool walk(typename ParameterTypeQualifier<KDL::Tree>::RefToConstT a_topology,
                            typename ParameterTypeQualifier<std::vector<typename OP::Param2T> >::RefToConstT a_jointStateVectorIn,
                            typename ParameterTypeQualifier<std::vector<typename OP::Param3T> >::RefToArgT a_linkStateVectorIn,
                            typename ParameterTypeQualifier<std::vector<typename OP::Param3T> >::RefToArgT a_linkStateVectorOut,
                            OP a_op)
    {

        //this is forward/outward iterative walk
        for (KDL::SegmentMap::const_iterator iter = a_topology.getSegments().begin(); iter != a_topology.getSegments().end(); ++iter)
        {
            const KDL::TreeElement currentElement = iter->second;
#ifdef VERBOSE_CHECK
//            std::cout << "Parent element name in current iteration " << currentElement.segment.getName() << std::endl;
//            std::cout << "State vector input " << a_linkStateVectorIn[currentElement.q_nr].segmentName << std::endl;
//            std::cout << "Parent joint index and value " << currentElement.q_nr << " " << a_jointStateVectorIn[currentElement.q_nr].q << std::endl;
#endif
            a_linkStateVectorOut[currentElement.q_nr] = a_linkStateVectorIn[currentElement.q_nr];

            for (std::vector<KDL::SegmentMap::const_iterator>::const_iterator childIter = iter->second.children.begin(); childIter != iter->second.children.end(); childIter++)
            {

#ifdef VERBOSE_CHECK
                std::cout << "Current element name in current iteration " << (*childIter)->second.segment.getName() << std::endl;
                std::cout << "Current joint index and value " << (*childIter)->second.q_nr << " " << a_jointStateVectorIn[(*childIter)->second.q_nr].q << std::endl;
#endif               
                a_linkStateVectorIn[(*childIter)->second.q_nr] = a_op(*childIter, a_jointStateVectorIn[(*childIter)->second.q_nr], a_linkStateVectorOut[currentElement.q_nr]);
            }
        }

        return true;
    };

    template <typename OP>
    inline static bool walk(typename ParameterTypeQualifier<KDL::Tree>::RefToConstT a_topology,
                            typename ParameterTypeQualifier<std::vector<typename OP::Param2T> >::RefToConstT a_jointStateVectorIn,
                            typename ParameterTypeQualifier<std::vector<typename OP::Param2T> >::RefToArgT a_jointStateVectorOut, //introduce a separate mutable state representation, now is used for testing
                            typename ParameterTypeQualifier<std::vector<typename OP::Param3T> >::RefToConstT a_linkStateVectorIn,
                            typename ParameterTypeQualifier<std::vector<typename OP::Param3T> >::RefToArgT a_linkStateVectorOut,
                            OP a_op)
    {

        return true;
    };

};

template<>
class DFSPolicy_ver2<KDL::Tree, inward>
{
public:

    DFSPolicy_ver2()
    {
    };

    ~DFSPolicy_ver2()
    {
    };

    template <typename OP>
    inline static bool walk(typename ParameterTypeQualifier<KDL::Tree>::RefToConstT a_topology,
                            typename ParameterTypeQualifier<std::vector<typename OP::Param2T> >::RefToConstT a_jointStateVectorIn,
                            typename ParameterTypeQualifier<std::vector<typename OP::Param3T> >::RefToArgT a_linkStateVectorIn,
                            typename ParameterTypeQualifier<std::vector<typename OP::Param3T> >::RefToArgT a_linkStateVectorOut,
                            OP a_op)
    {



        return true;
    };

    template <typename OP>
    inline static bool walk(typename ParameterTypeQualifier<KDL::Tree>::RefToConstT a_topology,
                            typename ParameterTypeQualifier<std::vector<typename OP::Param2T> >::RefToConstT a_jointStateVectorIn,
                            typename ParameterTypeQualifier<std::vector<typename OP::Param2T> >::RefToArgT a_jointStateVectorOut, //introduce a separate mutable state representation, now is used for testing
                            typename ParameterTypeQualifier<std::vector<typename OP::Param3T> >::RefToArgT a_linkStateVectorIn,
                            typename ParameterTypeQualifier<std::vector<typename OP::Param3T> >::RefToArgT a_linkStateVectorOut,
                            OP a_op)
    {

        SegmentState tempState;
#ifdef VERBOSE_CHECK
        std::cout << std::endl << "This is reverse iteration/inward" << std::endl;
#endif
        //this is reverse/inward iterative walk
        for (KDL::SegmentMap::const_reverse_iterator iter = a_topology.getSegments().rbegin(); iter != a_topology.getSegments().rend(); ++iter)
        {
            const KDL::TreeElement currentElement = iter->second;
            tempState = a_linkStateVectorIn[currentElement.q_nr];

#ifdef VERBOSE_CHECK

            std::cout << "Current element name in current reverse iteration " << currentElement.segment.getName() << std::endl;
            std::cout << "Current joint index and value in reverse iteration " << currentElement.q_nr << " " << a_jointStateVectorIn[currentElement.q_nr].q << std::endl;
            std::cout << "Spatial BODY force on a current element " << a_linkStateVectorIn[currentElement.q_nr].F << std::endl;
#endif
            //            TODO: make torque accessible. In order to do this we need to introduce mutable joint computational state.
            //            in total having 4 (2 immutable and mutable per link and per joint)
            //            also need to put this iteration into a separate reverse walk

            for (std::vector<KDL::SegmentMap::const_iterator>::const_iterator childIter = iter->second.children.begin(); childIter != iter->second.children.end(); childIter++)
            {

#ifdef VERBOSE_CHECK

                std::cout << "Child element name in current  reverse iteration " << (*childIter)->second.segment.getName() << std::endl;
                std::cout << "Child joint index and value " << (*childIter)->second.q_nr << " " << a_jointStateVectorIn[(*childIter)->second.q_nr].q << std::endl;
                std::cout << "Total spatial force on a child " << a_linkStateVectorOut[(*childIter)->second.q_nr].F << std::endl;
#endif
                //                torques(j--)=dot(S[i],f[i]);
                //                f[i - 1] = f[i - 1] + X[i] * f[i];
                //                the second term in the 2nd expression should be summed for all children of the parent and then added to the parent's force (ID for trees).
                //                a_linkStateVectorIn[parentElement.q_nr].F = a_linkStateVectorIn[parentElement.q_nr].F + a_linkStateVectorIn[(*childIter)->second.q_nr].X * a_linkStateVectorIn[(*childIter)->second.q_nr].F;
                //                double torque = dot(a_linkStateVectorIn[(*childIter)->second.q_nr].Z, a_linkStateVectorIn[(*childIter)->second.q_nr].F);

                //                    tempState = a_op(*childIter, a_jointStateVectorIn[(*childIter)->second.q_nr], tempState, a_linkStateVectorOut[(*childIter)->second.q_nr]); //uses operations with 4 args
                tempState = a_op(*childIter, a_jointStateVectorIn[(*childIter)->second.q_nr], a_jointStateVectorOut[(*childIter)->second.q_nr], tempState, a_linkStateVectorOut[(*childIter)->second.q_nr]); //uses operations with 5 args
                std::cout << "Torque on a current supporting joint " <<(*childIter)->second.q_nr << " " << a_jointStateVectorOut[(*childIter)->second.q_nr].torque << std::endl << std::endl;
            }
            //TODO: this needs to be changed, string comparison is inefficient
            if (iter->first != a_topology.getRootSegment()->first)
                a_linkStateVectorOut[currentElement.q_nr] = tempState;

#ifdef VERBOSE_CHECK
            std::cout << "Total Spatial force on a current element " << a_linkStateVectorOut[currentElement.q_nr].F << std::endl;
            std::cout << "Torque on a current supporting joint " << a_jointStateVectorOut[currentElement.q_nr].torque << std::endl << std::endl;
#endif

        }

        return true;
    };

};

};


#include "../../src/functionalcomputation_kdltypes.inl"
#endif	/* FUNCTIONALCOMPUTATION_KDLTYPES_HPP */

