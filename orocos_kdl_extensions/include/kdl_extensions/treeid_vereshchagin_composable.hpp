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
#include <boost/function.hpp>
#include <boost/bind.hpp>



//check Tine's work on semantic library for KDL
//to decouple these data structures.
//
namespace KDL
{

class SegmentState
{
public:
    SegmentState();
    SegmentState(const SegmentState& copy);
    SegmentState & operator=(const SegmentState& copy);
    KDL::Frame X;
    KDL::Twist Xdot;
    KDL::Twist Xdotdot;
    KDL::Wrench Fext;
    KDL::Twist Z; //supporting/driving joint unit twist/projection/Dof
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

class ForwardKinematicsComputation
{
public:
    ForwardKinematicsComputation(KDL::Twist& gravityAcc);
    SegmentState & operator() (const SegmentMap::const_iterator link,const JointState& js) ; //should declare the function and its arguments as constant
    SegmentState & operator() (const std::pair<std::string, KDL::TreeElement> link, const JointState& js) ;
    virtual ~ForwardKinematicsComputation();
private:
    SegmentState m_segmentstate;
    JointState m_jointstate;
    Twist m_gravity;
};

//primitive(atomic) function object
class ForceComputation
{
public:
    ForceComputation();
    KDL::Wrench & operator ()(const std::pair<std::string, const KDL::TreeElement> link, const SegmentState& ls) ;
    KDL::Wrench & operator ()(const SegmentMap::const_iterator link, const SegmentState& ls) ;
    virtual ~ForceComputation();
private:
    KDL::Wrench m_segmentforce;
};


boost::function<SegmentState& (SegmentMap::const_iterator linkIter, JointState& js)> computeForwardKinematicsComputation;

// function object adapter which composes primitive ones. compose_f_h(yz)_g(x)
//f ~= ForwardKinematicsComputationweep
//g ~= ForwardKinematicsComputation x ~= jointState
//h ~ =ForceComputation y ~= linkState output from g(x) and z ~= external forces/force input
template <typename OP1, typename OP2>
class Composer_f_gx : public std::unary_function<typename OP1::argument_type, typename OP2::result_type>
{
public:
    typedef typename OP2::result_type result_type;
    Composer_f_gx(const OP1& op1,const OP2& op2):a_op1(op1), a_op2(op2)
    {

    };
    result_type& operator()()
    {
        return;
    };
private:
    OP1 a_op1;
    OP2 a_op2;

};



}

#endif //~_TREEID_VERESHCHAGIN_COMPOSABLE_HPP_