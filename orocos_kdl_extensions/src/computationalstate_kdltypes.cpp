#include <kdl_extensions/computationalstate_kdltypes.hpp>

namespace kdle

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



//TODO: These has to be deprecated in the favor of Global and Local
//Link computational state

SegmentState::SegmentState()
{
    X.Identity();
    Xdot.Zero();
    Xdotdot.Zero();
    F.Zero();
    Fext.Zero();
    Z.Zero();
    Vj.Zero();
    jointIndex = 0;
    jointName = " ";
    segmentName = " ";

}

SegmentState::SegmentState(const SegmentState& copy)
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

SegmentState& SegmentState::operator=(const SegmentState& copy)
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

bool SegmentState::operator==(SegmentState& instance)
{
    return true;
};

SegmentState::~SegmentState()
{


}


//Joint computational state

JointState::JointState()
{
    q = 0.0;
    qdot = 0.0;
    qdotdot = 0.0;
    torque = 0.0;
    jointIndex = 0;
    jointName = " ";

}

JointState::JointState(const JointState& copy)
{
    q = copy.q;
    qdot = copy.qdot;
    qdotdot = copy.qdotdot;
    torque = copy.torque;
    jointIndex = copy.jointIndex;
    jointName = copy.jointName;
}

JointState& JointState::operator=(const JointState& copy)
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

JointState::~JointState()
{


}

};