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


//TODO: consider whether link local and global computational states
// should be represented in the single data type, as Herman suggested

//TODO: These has to be deprecated in the favor of Global and Local?
class SegmentState
{
public:
    SegmentState();
    SegmentState(const SegmentState& copy);
    SegmentState & operator=(const SegmentState& copy);
    bool operator==(const SegmentState& instance);
    bool operator!=(const SegmentState& instance);

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



class BaseOperation
{
public:

    BaseOperation()
    {
        a_segmentState.X.Identity();
        a_segmentState.Xdot.Zero();
        a_segmentState.Xdotdot.Zero();
    };

    virtual SegmentState & operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState)
    {
        return a_segmentState;
    };

    virtual SegmentState & operator()(const Segment& segmentId, const JointState& p_jointState, const SegmentState& p_segmentState)
    {
        return a_segmentState;
    };

    virtual ~BaseOperation()
    {
    };

    BaseOperation & operator=(const BaseOperation& copy)
    {
        if (this != &copy)
        {
            a_segmentState = copy.a_segmentState;
        }
        return *this;
    };

protected:
    SegmentState a_segmentState;

};


class transformPose : public BaseOperation
{
public:
    transformPose();
    //explicit transformPose(const SegmentState& p_segmentState);
    virtual ~transformPose(); //for templates this should be handled properly
     //just because KDL chain and tree use two different types one has to implement functions which take either map iterator or vector iterator.
    SegmentState& operator()(const KDL::Segment& segmentId, const JointState& p_jointState, SegmentState& p_segmentState);
    SegmentState& operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState);
   
};

class transformTwist : public BaseOperation
{
public:
    transformTwist();
    virtual ~transformTwist();
    SegmentState& operator()(const KDL::Segment& segmentId, const JointState& p_jointState, SegmentState& p_segmentState);
    SegmentState & operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState);

protected:
    KDL::Twist a_jointTwistVelocity;
    KDL::Twist a_jointUnitTwist; //motion subspace


};

class transformAccTwist : public BaseOperation
{
public:
    transformAccTwist();
    virtual ~transformAccTwist();
    SegmentState& operator()(const KDL::Segment& segmentId, const JointState& p_jointState, const SegmentState& p_segmentState);
    SegmentState & operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState);
};


class ProjectWrench: public BaseOperation
{
public:
    ProjectWrench();
    virtual ~ProjectWrench();


};

//composes functional computations/operations and return more complex computational operation
//helper class should be then hidden
// it should be able to cope with different combination of composition. Now only can handle
//two types
class compose
{
public:
    compose();
    compose(BaseOperation& p_op2, BaseOperation& p_op1) 
    {
        a_op1 = p_op1;
        a_op2 = p_op2;
    };
    virtual ~compose(){};
    SegmentState& operator()(const KDL::Segment& segmentId, const JointState& p_jointState,  SegmentState& p_segmentState)
    {
        a_segmentState = a_op2(segmentId, p_jointState, a_op1(segmentId, p_jointState, p_segmentState));
        return a_segmentState;
    }
    SegmentState & operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState)
    {
        a_segmentState = a_op2(segmentId, p_jointState, a_op1(segmentId, p_jointState, p_segmentState));
        return a_segmentState;
    };
    
    //    BaseOperation & operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState, transformTwist& p_computation2, transformPose& p_computation1);
    //    BaseOperation & operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState, transformPose& p_computation1, transformTwist& p_computation2);
protected:
    SegmentState a_segmentState;
    BaseOperation a_op1;
    BaseOperation a_op2;

};


class BaseIterationOperation
{
public:

    BaseIterationOperation()
    {
    };

    BaseIterationOperation(const BaseIterationOperation& copy)
    {
    };

    virtual ~BaseIterationOperation()
    {
    };

    BaseIterationOperation & operator=(const BaseIterationOperation& copy)
    {
        return *this;
    };
};

class iterateOverSegment : public BaseIterationOperation
{
protected:
    SegmentState a_segmentState;

public:
    iterateOverSegment();
    virtual ~iterateOverSegment();
    SegmentState & operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState, BaseOperation& p_computation);
    SegmentState & operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState, transformTwist& p_computation);
    SegmentState & operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState, transformAccTwist& p_computation);
    SegmentState & operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState, BaseOperation& p_computation2, BaseOperation& p_computation1);
    SegmentState & operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState, compose& p_computation);
    SegmentState & operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState, compose& p_computation, BaseOperation& p_computation1);
    SegmentState & operator()(SegmentMap::const_iterator segmentId, const JointState& p_jointState, const SegmentState& p_segmentState,  transformAccTwist& p_computation1, compose& p_computation);
};

class iterateOverTree : public BaseIterationOperation
{
public:

    iterateOverTree() : BaseIterationOperation()
    {
    };

    virtual ~iterateOverTree()
    {
    };
    //takes in a reference to an input vector of states and returns it modified
    // the problem with chain vs tree sucks. One uses vector and other map. The best option would be a hashtable which is equal in performance to a vector
    //for the time being lets use chain version.
    //we can still decide to parametrize this thing.
    bool operator()(KDL::Chain& p_tree, const std::vector<JointState>& p_jointState, std::vector<SegmentState>& p_segmentState, transformPose& p_computation);
    bool operator()(KDL::Chain& p_tree, const std::vector<JointState>& p_jointState, std::vector<SegmentState>& p_segmentState, transformTwist& p_computation);
    bool operator()(KDL::Chain& p_tree, const std::vector<JointState>& p_jointState, std::vector<SegmentState>& p_segmentState, transformTwist& p_computation2, transformPose& p_computation);
    bool operator()(KDL::Chain& p_tree, const std::vector<JointState>& p_jointState, std::vector<SegmentState>& p_segmentState, compose& p_computation);
    bool operator()(KDL::Chain& p_tree, const std::vector<JointState>& p_jointState, std::vector<SegmentState>& p_segmentState, compose& p_computation2, transformPose& p_computation1);
    //returns a vector of segmentstates
    std::vector<SegmentState> operator()(KDL::Chain& p_tree, const std::vector<JointState>& p_jointState, const std::vector<SegmentState>& p_segmentState, transformPose& p_computation);


};

typedef compose(*funcPtr)(transformTwist&, transformPose&);
typedef compose complexComputation;
typedef ProjectWrench projectWrench;
compose compose_ternary(transformTwist& op2, transformPose& op1);
compose compose_ternary(transformAccTwist& op2, transformTwist& op1);

}

#endif //~_TREEID_VERESHCHAGIN_COMPOSABLE_HPP_
