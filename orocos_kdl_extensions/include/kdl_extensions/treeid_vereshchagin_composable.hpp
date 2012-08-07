#ifndef _TREEID_VERESHCHAGIN_COMPOSABLE_HPP_
#define _TREEID_VERESHCHAGIN_COMPOSABLE_HPP_

#include <cstdlib>
#include <list>
#include <algorithm>
#include <functional>
#include <kdl/frames.hpp>
#include <kdl/joint.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>

namespace KDL
{

class SegmentState
{
public:
    SegmentState();
    SegmentState(const SegmentState& copy);
    SegmentState & operator=(const SegmentState& copy);

    Frame X;
    Twist Xdot;
    Twist Xdotdot;
    Wrench Fext;
    Twist Z; //supporting/driving joint unit twist/projection/Dof
    unsigned int jointIndex; // supporting/driving joint name/index
    std::string jointName;
    std::string segmentName;
    virtual ~SegmentState();
};

//immutable state

class JointState
{
public:
    JointState();
    JointState(const JointState& copy);
    JointState & operator=(const JointState& copy);
    double q;
    double qdot;
    double qdotdot;
    unsigned int jointIndex; //joint name/index
    std::string jointName;
    virtual ~JointState();
};

//primitive(atomic) function object
/*
class ForwardKinematics
{
public:
    ForwardKinematics(Twist& gravityAcc);
    SegmentState & operator() (SegmentMap::const_iterator link, JointState& js);
    SegmentState & operator() (std::pair<std::string, KDL::TreeElement> link, JointState& js);
    virtual ~ForwardKinematics();
private:
    SegmentState m_segmentstate;
    JointState m_jointstate;
    Twist m_gravity;
};

class ForceComputer
{
public:
    ForceComputer();
    KDL::Wrench & operator ()(std::pair<std::string, KDL::TreeElement> link, SegmentState& ls);
    KDL::Wrench & operator ()(SegmentMap::const_iterator link, SegmentState& ls);
    virtual ~ForceComputer();
private:
    KDL::Wrench m_segmentforce;
};


// function object adapter which composes primitive ones. compose_f_h(yz)_g(x)
//f ~= ForwardKinematicsComputationweep
//g ~= ForwardKinematicsComputation x ~= jointState
//h ~ =ForceComputation y ~= linkState output from g(x) and z ~= external forces/force input

template <typename Arg1, typename Arg2, typename Arg3, typename Result>
class ternary_function
{
    typedef Arg1 argument1_type;
    typedef Arg2 argument2_type;
    typedef Arg3 argument3_type;
    typedef Result result_type;

};
 */

// TODO transform function does propagation over single segment from root to tip, so does have initial segment always state zero
class transformPose
{
public:

    transformPose();
    //explicit transformPose(const SegmentState& p_segmentState);
    virtual ~transformPose(); //for templates this should be handled properly
    void operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState);
//private:
    SegmentState m_segmentState;
};

class transformTwist
{
public:
    transformTwist();
    virtual ~transformTwist();
    void operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState);
    SegmentState m_segmentState;
private:
    KDL::Twist a_jointTwistVelocity;
    KDL::Twist a_jointUnitTwist; //motion subspace


};
class compose;
class iterateOverSegment
{
public:
    iterateOverSegment();
    virtual ~iterateOverSegment();
    SegmentState& operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState, transformPose& p_computation);
    SegmentState& operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState, transformTwist& p_computation);
    SegmentState& operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState, transformPose& p_computation1, transformTwist& p_computation2);
    SegmentState& operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState, compose& p_computation);

};

//composes functional computations/operations and return more complex computational operation
class compose
{
public:
    compose();
    virtual ~compose();
    void operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState, transformPose& p_computation1, transformTwist& p_computation2);

};

//there should be another compose operation which composes the results of iterations (so values)

}

#endif //~_TREEID_VERESHCHAGIN_COMPOSABLE_HPP_
