#include "kdl_extensions/treeid_vereshchagin_composable.hpp"
//#define CHECK
namespace KDL
{

SegmentState::SegmentState()
{


}

SegmentState::SegmentState(const SegmentState& copy)
{
    X = copy.X;
    Xdot = copy.Xdot;
    Xdotdot = copy.Xdotdot;
    Z = copy.Z;
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
        Z = copy.Z;
        jointIndex = copy.jointIndex;
        jointName = copy.jointName;
        segmentName = copy.segmentName;
    }
    return *this;
}

SegmentState::~SegmentState()
{


}

JointState::JointState()
{


}

JointState::JointState(const JointState& copy)
{
    q = copy.q;
    qdot = copy.qdot;
    qdotdot = copy.qdotdot;
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
        jointIndex = copy.jointIndex;
        jointName = copy.jointName;
    }
    return *this;
}

JointState::~JointState()
{


}

transformPose::transformPose()
{
    m_segmentState.X.Identity();
    m_segmentState.Xdot.Zero();
    m_segmentState.Xdotdot.Zero();
}

//transformPose::transformPose(const SegmentState& p_segmentState)
//{
//    m_segmentState = p_segmentState;
//}

transformPose::~transformPose() //for templates this should be handled properly
{

}

void transformPose::operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState)
{
    //check for joint type None should be tree serialization function.
    
    m_segmentState.X = segmentId->second.segment.pose(p_jointState.q);
#ifdef CHECK
    std::cout << p_jointState.q<< std::endl;
    std::cout << segmentId->first<< std::endl;
    std::cout << segmentId->second.segment.pose(p_jointState.q) << std::endl;
#endif
    return ;
}


transformTwist::transformTwist()
{
    m_segmentState.X.Identity();
    m_segmentState.Xdot.Zero();
    m_segmentState.Xdotdot.Zero();
}

transformTwist::~transformTwist()
{



}


void transformTwist::operator ()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState)
{
    m_segmentState.X = segmentId->second.segment.pose(p_jointState.q);
    a_jointUnitTwist = m_segmentState.X.M.Inverse(segmentId->second.segment.twist(p_jointState.q, 1.0));
    a_jointTwistVelocity = m_segmentState.X.M.Inverse(segmentId->second.segment.twist(p_jointState.q, p_jointState.qdot));
    //do we check here for the index of a joint (whether the joint is first in the chain)
    m_segmentState.Xdot = m_segmentState.X.Inverse(p_segmentState.Xdotdot) + a_jointTwistVelocity;
    #ifdef CHECK
    std::cout << p_jointState.qdot<< std::endl;
    std::cout << "Xdot" << m_segmentState.Xdot << std::endl;
    #endif
    return;
}


compose::compose()
{

}

compose::~compose()
{

}


void compose::operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState, transformPose& p_computation1, transformTwist& p_computation2)
{

    return;
}



iterateOverSegment::iterateOverSegment()
{


}


iterateOverSegment::~iterateOverSegment()
{

}


SegmentState& iterateOverSegment::operator ()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState, transformPose& p_computation)
{
    p_computation(segmentId, p_jointState, p_segmentState);
#ifdef CHECK
    std::cout << p_jointState.q<< std::endl;
    std::cout << p_computation.m_segmentState.X << std::endl;
#endif
    return p_computation.m_segmentState;
}


SegmentState& iterateOverSegment::operator ()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState, transformTwist& p_computation)
{
    p_computation(segmentId, p_jointState, p_segmentState);
#ifdef CHECK
    std::cout << p_jointState.qdot<< std::endl;
    std::cout << "Xdot" << p_computation.m_segmentState.Xdot << std::endl;
#endif
    return p_computation.m_segmentState;
}


 SegmentState& iterateOverSegment::operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState, transformPose& p_computation1, transformTwist& p_computation2)
 {

     return p_computation1.m_segmentState;
 }

 SegmentState& iterateOverSegment::operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState, compose& p_computation)
 {

     //return
 }
/*
ForwardKinematics::ForwardKinematics(Twist& gravityAcc)
{
    m_gravity = gravityAcc;

    m_segmentstate.jointIndex = 0;
    //SetToZero(m_segmentstate.X);
    SetToZero(m_segmentstate.Xdot);
    SetToZero(m_segmentstate.Xdotdot);
    SetToZero(m_segmentstate.Z);

    m_jointstate.jointIndex = 0;
    m_jointstate.q = 0;
    m_jointstate.qdot = 0;
    m_jointstate.qdotdot = 0;
}

SegmentState& ForwardKinematics::operator()(SegmentMap::const_iterator link, JointState& js)
{
    m_segmentstate.X = link->second.segment.pose(js.q);
    Twist vj = m_segmentstate.X.M.Inverse(link->second.segment.twist(js.q, js.qdot));
    m_segmentstate.Z = m_segmentstate.X.M.Inverse(link->second.segment.twist(js.q, 1.0));

    if (link->second.q_nr == 0 || link->second.segment.getName() == "root")
    {
        m_segmentstate.jointIndex = link->second.q_nr;
        m_segmentstate.Xdot = vj;
        m_segmentstate.Xdotdot = m_segmentstate.X.Inverse(m_gravity) + m_segmentstate.Z * js.qdotdot + m_segmentstate.Xdot*vj;
    }
    else
    {

        //we need state dependent calculation
        m_segmentstate.Xdot = m_segmentstate.X.Inverse(m_segmentstate.Xdot) + vj;
        m_segmentstate.Xdotdot = m_segmentstate.X.Inverse(m_segmentstate.Xdotdot) + m_segmentstate.Z * js.qdotdot + m_segmentstate.Xdot*vj;
    }
    std::cout << "Inside: operator() - segment name" << link->first << std::endl;
    std::cout << "Inside: operator()- frame value" << m_segmentstate.X << std::endl;
    //std::cout << "Inside: operator()" << m_segmentstate.Xdot << std::endl;
    //std::cout << "Inside: operator()" << m_segmentstate.Xdotdot << std::endl;
    return m_segmentstate;
}

SegmentState& ForwardKinematics::operator ()(std::pair<std::string, KDL::TreeElement> link, JointState& js)
{
    m_segmentstate.X = link.second.segment.pose(js.q);
    Twist vj = m_segmentstate.X.M.Inverse(link.second.segment.twist(js.q, js.qdot));
    m_segmentstate.Z = m_segmentstate.X.M.Inverse(link.second.segment.twist(js.q, 1.0));

    if (link.second.q_nr == 0 || link.second.segment.getName() == "root")
    {
        m_segmentstate.jointIndex = link.second.q_nr;
        m_segmentstate.Xdot = vj;
        m_segmentstate.Xdotdot = m_segmentstate.X.Inverse(m_gravity) + m_segmentstate.Z * js.qdotdot + m_segmentstate.Xdot*vj;
    }
    else
    {

        //we need state dependent calculation
        m_segmentstate.Xdot = m_segmentstate.X.Inverse(m_segmentstate.Xdot) + vj;
        m_segmentstate.Xdotdot = m_segmentstate.X.Inverse(m_segmentstate.Xdotdot) + m_segmentstate.Z * js.qdotdot + m_segmentstate.Xdot*vj;
    }
    std::cout << "Inside: operator() - segment name" << link.first << std::endl;
    std::cout << "Inside: operator() - frame value" << m_segmentstate.X << std::endl;
    //std::cout << "Inside: operator()" << m_segmentstate.Xdot << std::endl;
    //std::cout << "Inside: operator()" << m_segmentstate.Xdotdot << std::endl;
    return m_segmentstate;


}

ForwardKinematics::~ForwardKinematics()
{

}

ForceComputer::ForceComputer()
{


}

KDL::Wrench& ForceComputer::operator ()(std::pair<std::string, KDL::TreeElement> link, SegmentState& ls)
{
    KDL::RigidBodyInertia I = link.second.segment.getInertia();
    m_segmentforce = I * ls.Xdotdot + ls.Xdot * (I * ls.Xdot) - ls.Fext;

    return m_segmentforce;
}

KDL::Wrench& ForceComputer::operator ()(SegmentMap::const_iterator link, SegmentState& ls)
{
     KDL::RigidBodyInertia I = link->second.segment.getInertia();
     m_segmentforce = I * ls.Xdotdot + ls.Xdot * (I * ls.Xdot) - ls.Fext;

    return m_segmentforce;
}

ForceComputer::~ForceComputer()
{


}

 */



}
