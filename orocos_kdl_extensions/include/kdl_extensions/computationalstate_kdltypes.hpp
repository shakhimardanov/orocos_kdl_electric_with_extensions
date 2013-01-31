/* 
 * File:   computationalstate_kdltypes.hpp
 * Author: azamat
 *
 * Created on September 28, 2012, 4:36 PM
 */

#ifndef COMPUTATIONALSTATE_KDLTYPES_HPP
#define	COMPUTATIONALSTATE_KDLTYPES_HPP

#include <kdl/frames.hpp>
#include <Pose/Pose.h>
#include <Twist/Twist.h>
#include <Wrench/Wrench.h>

namespace grs = geometric_semantics;
// TODO: maybe we should introduce domain independent
// representation of a computational state and instantiate for the specific domain
// The specific case could looks as below.

namespace kdle
{

//TODO: consider whether link local and global computational states
// should be represented in the single data type, as Herman suggested
class SegmentLocalCompState
{
public:
    SegmentLocalCompState();
    SegmentLocalCompState(const SegmentLocalCompState& copy);
    SegmentLocalCompState & operator=(const SegmentLocalCompState& copy);

    grs::Pose<KDL::Frame> X;
    grs::Twist<KDL::Twist> Xdot;
    grs::Twist<KDL::Twist> Xdotdot;
    grs::Wrench<KDL::Wrench> Fext;
    grs::Wrench<KDL::Wrench> F;
    KDL::Twist Z; //supporting/driving joint unit twist/projection/Dof
    KDL::Twist Vj;
    unsigned int jointIndex; // supporting/driving joint name/index
    std::string jointName;
    std::string segmentName;
    virtual ~SegmentLocalCompState();
};


class SegmentGlobalCompState
{
public:
    SegmentGlobalCompState();
    SegmentGlobalCompState(const SegmentGlobalCompState& copy);
    SegmentGlobalCompState & operator=(const SegmentGlobalCompState& copy);

    grs::PoseCoordinates<KDL::Frame> X;
    KDL::Twist Xdot;
    KDL::Twist Xdotdot;
    KDL::Wrench Fext;
    KDL::Wrench F;
    KDL::Twist Z; //supporting/driving joint unit twist/projection/Dof
    KDL::Twist Vj;
    unsigned int jointIndex; // supporting/driving joint name/index
    std::string jointName;
    std::string segmentName;
    virtual ~SegmentGlobalCompState();
};
//immutable state

class JointLocalCompState
{
public:
    JointLocalCompState();
    JointLocalCompState(const JointLocalCompState& copy);
    JointLocalCompState & operator=(const JointLocalCompState& copy);
    double q;
    double qdot;
    double qdotdot;
    double torque;
    unsigned int jointIndex; //joint name/index
    std::string jointName;
    virtual ~JointLocalCompState();
};

//TODO: These has to be deprecated in the favor of Global and Local
class SegmentState
{
public:
    SegmentState();
    SegmentState(const SegmentState& copy);
    SegmentState & operator=(const SegmentState& copy);
    bool operator==(SegmentState& instance);

    KDL::Frame X;
    KDL::Twist Xdot;
    KDL::Twist Xdotdot;
    KDL::Wrench Fext;
    KDL::Wrench F;
    KDL::Twist Z; //supporting/driving joint unit twist/projection/Dof
    KDL::Twist Vj;
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
    double torque;
    unsigned int jointIndex; //joint name/index
    std::string jointName;
    virtual ~JointState();
};


};


#endif	/* COMPUTATIONALSTATE_KDLTYPES_HPP */

