#include "kdl_extensions/treeid_vereshchagin_composable.hpp"
//#define CHECK
namespace KDL
{

//Link computational state

SegmentState::SegmentState()
{
    X.Identity();
    Xdot.Zero();
    Xdotdot.Zero();
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
        Z = copy.Z;
        Vj = copy.Vj;
        jointIndex = copy.jointIndex;
        jointName = copy.jointName;
        segmentName = copy.segmentName;
    }
    return *this;
}

SegmentState::~SegmentState()
{


}


//Joint computational state

JointState::JointState()
{
    q = 0.0;
    qdot = 0.0;
    qdotdot = 0.0;
    jointIndex = 0;
    jointName = " ";

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

//Transform operations

transformPose::transformPose() : BaseOperation()
{

}

transformPose::~transformPose() //for templates this should be handled properly
{

}

SegmentState& transformPose::operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState)
{
    //check for joint type None should be tree serialization function.

    //a_segmentState.X =  p_segmentState.X * segmentId->second.segment.pose(p_jointState.q); //in base coordinates
    a_segmentState.Xdot = p_segmentState.Xdot;
    a_segmentState.Xdotdot = p_segmentState.Xdotdot;
    a_segmentState.X = segmentId->second.segment.pose(p_jointState.q);
    a_segmentState.jointIndex = p_jointState.jointIndex;
    a_segmentState.jointName = p_jointState.jointName;
    a_segmentState.segmentName = segmentId->first;
    std::cout << "Inside transformPose 0" << a_segmentState.Xdot << std::endl;

#ifdef CHECK
    std::cout << p_jointState.q << std::endl;
    std::cout << segmentId->first << std::endl;
    std::cout << segmentId->second.segment.pose(p_jointState.q) << std::endl;
#endif
    return a_segmentState;
}

SegmentState& transformPose::operator()(const KDL::Segment& segmentId, const JointState& p_jointState, SegmentState& p_segmentState)
{
    //a_segmentState.X = p_segmentState.X * segmentId.pose(p_jointState.q); //in base coordinates
    a_segmentState.Xdot = p_segmentState.Xdot;
    a_segmentState.Xdotdot = p_segmentState.Xdotdot;
    a_segmentState.X = segmentId.pose(p_jointState.q);
    a_segmentState.jointIndex = p_jointState.jointIndex;
    a_segmentState.jointName = p_jointState.jointName;
    a_segmentState.segmentName = segmentId.getName();

    std::cout << "Inside transformPose 1" << a_segmentState.Xdot << std::endl;
    return a_segmentState;

}

transformTwist::transformTwist() : BaseOperation()
{

}

transformTwist::~transformTwist()
{



}

//pre-condition transformPose should be called first?

SegmentState& transformTwist::operator ()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState)
{
    //To do: how do we check that this opeation is applied stand alone or depends on a previous operation which it was composed with.
    //should we provide two implementations which will be called according to this separate cases.
    //Otherwise, if this operation is not aware that it is being used in combination with another one. we will be re-doing the operation twice once in
    //transformPose and once in this operation.
    //in current KDL iterations are done multiple times
    //in my impl computations will be done multiple times.
    //to resolve this issue in this impl. preconditions for the computations must be set. Such transformTwist should only be called with the output of transformPose,
    //and transformAcc should be called with output of transformTwist. Mathematically: f_acc(f_vel(f_pose(jointState)))
    /*
        //version that is context detached and does not require preconditon transformPose.
        a_segmentState.X = segmentId->second.segment.pose(p_jointState.q);
        //if ((p_jointState.jointIndex==p_segmentState.jointIndex) && (a_segmentState.X == p_segmentState.X))
        a_jointUnitTwist = a_segmentState.X.M.Inverse(segmentId->second.segment.twist(p_jointState.q, 1.0));
        a_segmentState.Z = a_jointUnitTwist;
        a_jointTwistVelocity = a_segmentState.X.M.Inverse(segmentId->second.segment.twist(p_jointState.q, p_jointState.qdot));
        //do we check here for the index of a joint (whether the joint is first in the chain)
        a_segmentState.Xdot = a_segmentState.X.Inverse(p_segmentState.Xdotdot) + a_jointTwistVelocity;
     */
    //version that requires transformPoses output.
    a_segmentState.X = p_segmentState.X;
    a_segmentState.Xdotdot = p_segmentState.Xdotdot;
    a_segmentState.Z = a_segmentState.X.M.Inverse(segmentId->second.segment.twist(p_jointState.q, 1.0));
    //a_segmentState.Z = a_jointUnitTwist;
    a_segmentState.Vj = a_segmentState.X.M.Inverse(segmentId->second.segment.twist(p_jointState.q, p_jointState.qdot));
    //a_segmentState.Vj = a_jointTwistVelocity;

    //do we check here for the index of a joint (whether the joint is first in the chain)
    a_segmentState.Xdot = a_segmentState.X.Inverse(p_segmentState.Xdot) + a_segmentState.Vj;

#ifdef CHECK
    std::cout << p_jointState.qdot << std::endl;
    std::cout << "Xdot" << m_segmentState.Xdot << std::endl;
#endif
    return a_segmentState;
}

SegmentState& transformTwist::operator()(const KDL::Segment& segmentId, const JointState& p_jointState, SegmentState& p_segmentState)
{
    a_segmentState.X = p_segmentState.X;
    a_segmentState.Xdotdot = p_segmentState.Xdotdot;
    a_segmentState.Z = a_segmentState.X.M.Inverse(segmentId.twist(p_jointState.q, 1.0));
    //a_segmentState.Z = a_jointUnitTwist;
    a_segmentState.Vj = a_segmentState.X.M.Inverse(segmentId.twist(p_jointState.q, p_jointState.qdot));
    //a_segmentState.Vj = a_jointTwistVelocity;
    //do we check here for the index of a joint (whether the joint is first in the chain)
    a_segmentState.Xdot = a_segmentState.X.Inverse(p_segmentState.Xdot) + a_segmentState.Vj;

    return a_segmentState;
}

transformAccTwist::transformAccTwist() : BaseOperation()
{

}

transformAccTwist::~transformAccTwist()
{

}

SegmentState& transformAccTwist::operator ()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState)
{
    //pre-condition is that this transform is called with the output from transformTwist
    a_segmentState = p_segmentState;
    a_segmentState.Xdotdot = p_segmentState.X.Inverse(p_segmentState.Xdotdot) + p_segmentState.Z * p_jointState.qdotdot + p_segmentState.Xdot * p_segmentState.Vj;
    return a_segmentState;
}

SegmentState& transformAccTwist::operator()(const KDL::Segment& segmentId, const JointState& p_jointState, const SegmentState& p_segmentState)
{
    a_segmentState = p_segmentState;
    a_segmentState.Xdotdot = p_segmentState.X.Inverse(p_segmentState.Xdotdot) + p_segmentState.Z * p_jointState.qdotdot + p_segmentState.Xdot * p_segmentState.Vj;
    return a_segmentState;
}

//Composition operation

compose::compose() : BaseOperation()
{

}

compose::compose(transformTwist& p_op2, transformPose& p_op1) : BaseOperation()
{
    a_op1 = p_op1;
    a_op2 = p_op2;
}

compose::compose(transformAccTwist& p_op2, transformTwist& p_op1) : BaseOperation()
{
    a_op2 = p_op1;
    a_op3 = p_op2;
}

compose::~compose()
{

}

//there should be another compose operation which composes the results of iterations (so values)

compose compose_ternary(transformTwist& op2, transformPose& op1)
{

    return compose(op2, op1);
};

compose compose_ternary(transformAccTwist& op2, transformTwist& op1)
{
    return compose(op2, op1);
};
/*
BaseOperation& compose::operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState, transformTwist& p_computation2, transformPose& p_computation1)
{

    p_computation2(segmentId, p_jointState, p_computation1(segmentId, p_jointState, p_segmentState));
    return p_computation2;
}

BaseOperation& compose::operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState, transformPose& p_computation1, transformTwist& p_computation2)
{
    p_computation2(segmentId, p_jointState, p_computation1(segmentId, p_jointState, p_segmentState));
    return p_computation2;
}
 */

//Iteratino operation

iterateOverSegment::iterateOverSegment() : BaseIterationOperation()
{
    a_segmentState.X.Identity();
    a_segmentState.Xdot.Zero();
    a_segmentState.Xdotdot.Zero();

}

iterateOverSegment::~iterateOverSegment()
{

}

SegmentState& iterateOverSegment::operator ()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState, transformPose& p_computation)
{
    a_segmentState = p_computation(segmentId, p_jointState, p_segmentState);
#ifdef CHECK
    std::cout << p_jointState.q << std::endl;
    std::cout << p_computation.m_segmentState.X << std::endl;
#endif
    return a_segmentState;
}

SegmentState& iterateOverSegment::operator ()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState, transformTwist& p_computation)
{
    a_segmentState = p_computation(segmentId, p_jointState, p_segmentState);
#ifdef CHECK
    std::cout << p_jointState.qdot << std::endl;
    std::cout << "Xdot" << p_computation.m_segmentState.Xdot << std::endl;
#endif
    return a_segmentState;
}

SegmentState & iterateOverSegment::operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState, transformAccTwist& p_computation)
{
    a_segmentState = p_computation(segmentId, p_jointState, p_segmentState);
    return a_segmentState;
}

SegmentState& iterateOverSegment::operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState, transformTwist& p_computation2, transformPose& p_computation1)
{

    a_segmentState = p_computation2(segmentId, p_jointState, p_computation1(segmentId, p_jointState, p_segmentState));
    return a_segmentState;
}

SegmentState& iterateOverSegment::operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState, compose& p_computation)
{

    return p_computation(segmentId, p_jointState, p_segmentState);
}

SegmentState & iterateOverSegment::operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState, compose& p_computation, transformPose& p_computation1)
{

    return p_computation(segmentId, p_jointState, p_computation1(segmentId, p_jointState, p_segmentState));
}

SegmentState & iterateOverSegment::operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState, transformAccTwist& p_computation1, compose& p_computation)
{
    return p_computation1(segmentId, p_jointState, p_computation(segmentId, p_jointState, p_segmentState));
}


//takes in a reference to an input vector of states and returns it modified

bool iterateOverTree::operator()(KDL::Chain& p_tree, const std::vector<JointState>& p_jointState, std::vector<SegmentState>& p_segmentState, transformPose& p_computation)
{

    //where should the check for joint None happen
    // does it make sense to tie topology with the state info. for instance at each node with a reference to a state
    //representations for state should be stored in the same data container as the one used for the chain or tree. In our case vectors or maps.
    // maps would have been the best choice but the problem is the performance penalty one has to pay for most of the map operations.
    //current dirty implementation will rely on overloading the same functor operator() so that it takes both map type and vector type iterators.
    //for ( std::vector<Segment>::const_iterator treeIterator = p_tree.segments.begin(); treeIterator != p_tree.segments.end(); treeIterator++)
    for (unsigned int segmentId = 0; segmentId < p_tree.getNrOfSegments(); segmentId++)
    {
        if (segmentId != 0)
            p_segmentState[segmentId] = p_computation(p_tree.getSegment(segmentId), p_jointState[segmentId], p_segmentState[segmentId - 1]);
        else
            p_segmentState[segmentId] = p_computation(p_tree.getSegment(segmentId), p_jointState[segmentId], p_segmentState[segmentId]);
        //this loop should compute values in base/global coordinates. Need to change, computations only support link local
        //should the transforms be included here?
    }
    return true;

}

bool iterateOverTree::operator()(KDL::Chain& p_tree, const std::vector<JointState>& p_jointState, std::vector<SegmentState>& p_segmentState, transformTwist& p_computation)
{
    for (unsigned int segmentId = 0; segmentId < p_tree.getNrOfSegments(); segmentId++)
    {
        if (segmentId != 0)
            p_segmentState[segmentId] = p_computation(p_tree.getSegment(segmentId), p_jointState[segmentId], p_segmentState[segmentId - 1]);
        else
            p_segmentState[segmentId] = p_computation(p_tree.getSegment(segmentId), p_jointState[segmentId], p_segmentState[segmentId]);
    }
    return true;
}

bool iterateOverTree::operator()(KDL::Chain& p_tree, const std::vector<JointState>& p_jointState, std::vector<SegmentState>& p_segmentState, transformTwist& p_computation2, transformPose& p_computation)
{
    for (unsigned int segmentId = 0; segmentId < p_tree.getNrOfSegments(); segmentId++)
    {
        if (segmentId != 0)
            p_segmentState[segmentId] = p_computation2(p_tree.getSegment(segmentId), p_jointState[segmentId], p_computation(p_tree.getSegment(segmentId), p_jointState[segmentId], p_segmentState[segmentId - 1]));
        else
            p_segmentState[segmentId] = p_computation2(p_tree.getSegment(segmentId), p_jointState[segmentId], p_computation(p_tree.getSegment(segmentId), p_jointState[segmentId], p_segmentState[segmentId]));

    }
    return true;
}

bool iterateOverTree::operator()(KDL::Chain& p_tree, const std::vector<JointState>& p_jointState, std::vector<SegmentState>& p_segmentState, compose& p_computation)
{
    for (unsigned int segmentId = 0; segmentId < p_tree.getNrOfSegments(); segmentId++)
    {

        if (segmentId != 0)
            p_segmentState[segmentId] = p_computation(p_tree.getSegment(segmentId), p_jointState[segmentId], p_segmentState[segmentId - 1]);
        else
            p_segmentState[segmentId] = p_computation(p_tree.getSegment(segmentId), p_jointState[segmentId], p_segmentState[segmentId]);
        //this loop should compute values in base/global coordinates. Need to change.

    }


    return true;
}

bool iterateOverTree::operator()(KDL::Chain& p_tree, const std::vector<JointState>& p_jointState, std::vector<SegmentState>& p_segmentState, compose& p_computation2, transformPose& p_computation1)
{
    for (unsigned int segmentId = 0; segmentId < p_tree.getNrOfSegments(); segmentId++)
    {

        if (segmentId != 0)
        {
            p_segmentState[segmentId] = p_computation2(p_tree.getSegment(segmentId), p_jointState[segmentId], p_computation1(p_tree.getSegment(segmentId), p_jointState[segmentId], p_segmentState[segmentId - 1]));
        }
        else
            p_segmentState[segmentId] = p_computation2(p_tree.getSegment(segmentId), p_jointState[segmentId], p_computation1(p_tree.getSegment(segmentId), p_jointState[segmentId], p_segmentState[segmentId]));
        //this loop should compute values in base/global coordinates. Need to change.

    }


    return true;
}

//returns a vector of segmentstates

std::vector<SegmentState> iterateOverTree::operator()(KDL::Chain& p_tree, const std::vector<JointState>& p_jointState, const std::vector<SegmentState>& p_segmentState, transformPose& p_computation)
{

    return p_segmentState;
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
