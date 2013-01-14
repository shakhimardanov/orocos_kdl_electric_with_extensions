#include <kdl_extensions/computationalstate_kdltypes.hpp>

namespace kdl_extensions

{
//Link computational state

SegmentLocalCompState::SegmentLocalCompState()
{


}

SegmentLocalCompState::SegmentLocalCompState(const SegmentLocalCompState& copy)
{
    X = copy.X;
    Xdot = copy.Xdot;
    Xdotdot = copy.Xdotdot;
    F = copy.F;
    Z = copy.Z;
    Vj = copy.Vj;
    jointIndex = copy.jointIndex;
    jointName = copy.jointName;
    segmentName = copy.segmentName;
}

SegmentLocalCompState& SegmentLocalCompState::operator=(const SegmentLocalCompState& copy)
{
    if (this != &copy)
    {
        X = copy.X;
        Xdot = copy.Xdot;
        Xdotdot = copy.Xdotdot;
        F = copy.F;
        Z = copy.Z;
        Vj = copy.Vj;
        jointIndex = copy.jointIndex;
        jointName = copy.jointName;
        segmentName = copy.segmentName;
    }
    return *this;
}

SegmentLocalCompState::~SegmentLocalCompState()
{


}


//Joint computational state

JointLocalCompState::JointLocalCompState()
{
    q = 0.0;
    qdot = 0.0;
    qdotdot = 0.0;
    torque = 0.0;
    jointIndex = 0;
    jointName = " ";

}

JointLocalCompState::JointLocalCompState(const JointLocalCompState& copy)
{
    q = copy.q;
    qdot = copy.qdot;
    qdotdot = copy.qdotdot;
    torque = copy.torque;
    jointIndex = copy.jointIndex;
    jointName = copy.jointName;
}

JointLocalCompState& JointLocalCompState::operator=(const JointLocalCompState& copy)
{
    if (this != &copy)
    {
        q = copy.q;
        qdot = copy.qdot;
        qdotdot = copy.qdotdot;
        torque = copy.torque;
        jointIndex = copy.jointIndex;
        jointName = copy.jointName;
    }
    return *this;
}

JointLocalCompState::~JointLocalCompState()
{


}

};