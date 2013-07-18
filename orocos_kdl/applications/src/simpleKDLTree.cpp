/* 
 * File:   simpleKDLTree.cpp
 * Author: azamat
 *
 * Created on December 21, 2011, 11:46 AM
 */

#include <cstdlib>
#include <frames.hpp>
#include <joint.hpp>
#include <frames_io.hpp>
#include <chainfksolverpos_recursive.hpp>
#include <chainiksolverpos_nr.hpp>
#include <chain.hpp>
#include <tree.hpp>
//forward pose kinematics solver.
//defines ik vel level weighted damped least square solver
#include <treeiksolvervel_wdls.hpp>
//generates jacobian for a tree given its joint values
#include <treejnttojacsolver.hpp>
#include <treefksolverpos_recursive.hpp>
#include <treeidsolver_vereshchagin.hpp>


using namespace std;

int main(int argc, char** argv)
{
    using namespace KDL;
    Joint joint1 = Joint("j1", Joint::RotZ, 1, 0, 0.01);
    Joint joint2 = Joint("j2", Joint::RotZ, 1, 0, 0.01);
    Joint joint3 = Joint("j3", Joint::RotZ, 1, 0, 0.01);
    Joint joint4 = Joint("j4", Joint::RotZ, 1, 0, 0.01);
    Joint joint5 = Joint("j5", Joint::RotZ, 1, 0, 0.01);
    Joint joint6 = Joint("j6", Joint::RotZ, 1, 0, 0.01);
    Joint joint7 = Joint("j7", Joint::RotZ, 1, 0, 0.01);
    Joint joint8 = Joint("j8", Joint::RotZ, 1, 0, 0.01);
    Joint joint9 = Joint("j9", Joint::RotZ, 1, 0, 0.01);

    Frame frame1(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
    Frame frame2(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
    Frame frame3(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
    Frame frame4(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
    Frame frame5(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
    Frame frame6(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
    Frame frame7(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
    Frame frame8(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
    Frame frame9(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));

    //Segment (const Joint &joint=Joint(Joint::None), const Frame &f_tip=Frame::Identity(), const RigidBodyInertia &I=RigidBodyInertia::Zero())
    Segment segment1 = Segment("L1", joint1, frame1);
    Segment segment2 = Segment("L2", joint2, frame2);
    Segment segment3 = Segment("L3", joint3, frame3);
    Segment segment4 = Segment("L4", joint4, frame4);
    Segment segment5 = Segment("L5", joint5, frame5); 
    Segment segment6 = Segment("L6", joint6, frame6);
    Segment segment7 = Segment("L7", joint7, frame7);
    Segment segment8 = Segment("L8", joint8, frame8);
    Segment segment9 = Segment("L9", joint9, frame9);
    // 	RotationalInertia (double Ixx=0, double Iyy=0, double Izz=0, double Ixy=0, double Ixz=0, double Iyz=0)
    RotationalInertia rotInerSeg1(0.0, 0.0, 0.0, 0.0, 0.0, 0.0); //around symmetry axis of rotation
    double pointMass = 0.3; //in kg
    //RigidBodyInertia (double m=0, const Vector &oc=Vector::Zero(), const RotationalInertia &Ic=RotationalInertia::Zero())
    RigidBodyInertia inerSegment1(pointMass, Vector(0.0, -0.4, 0.0), rotInerSeg1);
    RigidBodyInertia inerSegment2(pointMass, Vector(0.0, -0.4, 0.0), rotInerSeg1);
    RigidBodyInertia inerSegment3(pointMass, Vector(0.0, -0.4, 0.0), rotInerSeg1);
    RigidBodyInertia inerSegment4(pointMass, Vector(0.0, -0.4, 0.0), rotInerSeg1);
    RigidBodyInertia inerSegment5(pointMass, Vector(0.0, -0.4, 0.0), rotInerSeg1);
    RigidBodyInertia inerSegment6(pointMass, Vector(0.0, -0.4, 0.0), rotInerSeg1);
    RigidBodyInertia inerSegment7(pointMass, Vector(0.0, -0.4, 0.0), rotInerSeg1);
    RigidBodyInertia inerSegment8(pointMass, Vector(0.0, -0.4, 0.0), rotInerSeg1);
    RigidBodyInertia inerSegment9(pointMass, Vector(0.0, -0.4, 0.0), rotInerSeg1);

    segment1.setInertia(inerSegment1);
    segment2.setInertia(inerSegment2);
    segment3.setInertia(inerSegment3);
    segment4.setInertia(inerSegment4);
    segment5.setInertia(inerSegment5);
    segment6.setInertia(inerSegment6);
    segment7.setInertia(inerSegment7);
    segment8.setInertia(inerSegment8);
    segment9.setInertia(inerSegment9);

    Tree twoBranchTree("L0");

    twoBranchTree.addSegment(segment1, "L0");
    twoBranchTree.addSegment(segment2, "L1");
    twoBranchTree.addSegment(segment3, "L2");
    twoBranchTree.addSegment(segment4, "L3");
    twoBranchTree.addSegment(segment5, "L2");   //branches connect at joint 3
    twoBranchTree.addSegment(segment6, "L5");
    twoBranchTree.addSegment(segment7, "L6");
    twoBranchTree.addSegment(segment8, "L6");
    twoBranchTree.addSegment(segment9, "L8");

    JntArray q(twoBranchTree.getNrOfJoints());
    /*
    std::cout << "Number of joints " << twoBranchTree.getNrOfJoints() << std::endl;
    std::cout << "Number of segments " << twoBranchTree.getNrOfSegments() << std::endl;
    SegmentMap::const_iterator iter;
    SegmentMap::const_iterator iterRoot;
    int i = 0;
    std::string parentName = "";
    std::string branchingLinkName = "";
    Chain chainInTree[3];
    iterRoot = twoBranchTree.getRootSegment();
    for (iter = twoBranchTree.getSegments().begin(); iter != twoBranchTree.getSegments().end(); iter++)
    {
        //std::cout << iter->second.parent->first << std::endl;
        if (iter->second.children.size() > 1)
        {
            branchingLinkName = iter->first;
            std::cout << "branching "<<iter->first << std::endl;
        }
        if (iter->second.children.size() == 0)
        {
            std::cout << iter->first << std::endl;
            //std::string rootName = iterRoot->second.segment.getName();
            //std::string tipName = iter->second.segment.getName();
            twoBranchTree.getChain(iterRoot->first, iter->first, chainInTree[i]);
            std::cout << "Number of chain in the tree " << i << std::endl;
            std::cout << "Number of chain joints in branch " << chainInTree[i].getNrOfJoints() << std::endl;
            std::cout << "Number of chain links in  branch " << chainInTree[i].getNrOfSegments() << std::endl;
            std::cout << chainInTree[i].getSegment(1).getName() << std::endl;
            std::cout << chainInTree[i].getSegment(1).getJoint().getName() << std::endl;

            i++;
        }


    }

    TreeFkSolverPos_recursive cartPoseSolver(twoBranchTree);
    //q(2) = M_PI / 3.0;
    Frame cartPosition;
    cartPoseSolver.JntToCart(q, cartPosition, "L7");
    std::cout << "For L7 link of the tree " << cartPosition << std::endl;

    JntArray q_tilde(chainInTree[0].getNrOfJoints());
   // q_tilde(3) = M_PI / 3.0;
    ChainFkSolverPos_recursive cartPoseSolver2(chainInTree[0]);
    int temp = cartPoseSolver2.JntToCart(q_tilde, cartPosition, 4); // for chains you have to get it through segment number and for tree with the name
    std::cout << "For L4 link of the chain " << cartPosition << std::endl; // what is q_nr in TreeElement and what is the physical meaning of "root".


    std::cout << "=========================================================" << std::endl;
*/
     //arm root acceleration
    Vector linearAcc(0.0, 10, 0.0); //gravitational acceleration along Y
    Vector angularAcc(0.0, 0.0, 0.0);
    Twist rootAcc(linearAcc, angularAcc);
    //arm joint rate configuration and initial values are zero
    JntArray qDot(twoBranchTree.getNrOfJoints());
    //arm joint accelerations returned by the solver
    JntArray qDotDot(twoBranchTree.getNrOfJoints());
    //arm joint torques returned by the solver
    JntArray qTorque(twoBranchTree.getNrOfJoints());

     //external forces on the arm
    Vector externalForce1(0.0, 0.0, 0.0);
    Vector externalTorque1(0.0, 0.0, 0.0);
    Vector externalForce2(0.0, 0.0, 0.0);
    Vector externalTorque2(0.0, 0.0, 0.0);
    Wrench externalNetForce1(externalForce1, externalTorque1);
    Wrench externalNetForce2(externalForce2, externalTorque2);
    Wrenches externalNetForce;
    externalNetForce.push_back(externalNetForce1);

    Vector constrainXLinear(0.0, 0.0, 0.0);
    Vector constrainXAngular(0.0, 0.0, 0.0);
    Vector constrainYLinear(0.0, 0.0, 0.0);
    Vector constrainYAngular(0.0, 0.0, 0.0);
    Vector constrainZLinear(0.0, 0.0, 0.0);
    Vector constrainZAngular(0.0, 0.0, 0.0);
    const Twist constraintForcesX(constrainXLinear, constrainXAngular);
    const Twist constraintForcesY(constrainYLinear, constrainYAngular);
    const Twist constraintForcesZ(constrainZLinear, constrainZAngular);
    Jacobian alpha(1);
    alpha.setColumn(0, constraintForcesX);
    //alpha.setColumn(1,constraintForcesY);
    //alpha.setColumn(2,constraintForcesZ);

    //Acceleration energy at  the end-effector
    JntArray betha(1); //set to zero
    betha(0) = 0.0;
    q(0) = 0.0; //M_PI/2.0;
    q(2) = 0.0; //M_PI/2.0;
    q(4) = 0.0;//M_PI/3.0;
    qDot(0) = 0.0;//2.0;

    std::cout << "j0/L1 " << q(0) << " j2/L3 " << q(2) << " j4/L5 " << q(4) << std::endl;
    
    TreeIdSolver_Vereshchagin idSolver(twoBranchTree, "L4", rootAcc,1);
    idSolver.CartToJnt(q, qDot, qDotDot, alpha, betha, externalNetForce, qTorque);

    q(0) = M_PI/3.0;
    q(2) = M_PI/3.0;
    q(4) = M_PI/4.0;
    qDot(0) = 3.0;
    idSolver.CartToJnt(q, qDot, qDotDot, alpha, betha, externalNetForce, qTorque);


    return 0;
}



