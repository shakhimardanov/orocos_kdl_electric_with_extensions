#include "orocos_kdl_extensions/treeid_vereshchagin_composable.hpp"

namespace KDL
{

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


    KDL::Wrench& ForceComputer::operator ()(std::pair<std::string,KDL::TreeElement> link, SegmentState& ls)
    {
        KDL::RigidBodyInertia I=link.second.segment.getInertia();
        m_segmentforce =I*ls.Xdotdot + ls.Xdot*(I*ls.Xdot)-ls.Fext;

        return m_segmentforce;
    }

     KDL::Wrench& ForceComputer::operator ()(SegmentMap::const_iterator link, SegmentState& ls)
    {
        KDL::RigidBodyInertia I=link->second.segment.getInertia();
        m_segmentforce =I*ls.Xdotdot + ls.Xdot*(I*ls.Xdot)-ls.Fext;

        return m_segmentforce;
    }

     ForceComputer::~ForceComputer()
     {


     }


     SegmentState& forwardKinematicSweep(SegmentMap::const_iterator first1, SegmentMap::const_iterator second1, JointState& first2, SegmentState& first3)
     {
         SegmentState result;

         return result;
     }



}
