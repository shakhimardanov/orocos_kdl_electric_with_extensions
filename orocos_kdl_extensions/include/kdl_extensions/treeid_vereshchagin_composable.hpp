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





SegmentState& forwardKinematicSweep(SegmentMap::const_iterator first1, SegmentMap::const_iterator second1, JointState& first2, SegmentState& first3);




}

#endif //~_TREEID_VERESHCHAGIN_COMPOSABLE_HPP_