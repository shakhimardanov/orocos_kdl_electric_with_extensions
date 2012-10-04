/* 
 * File:   simpleKDLTree.cpp
 * Author: azamat
 *
 * Created on December 21, 2011, 11:46 AM
 */



#include <cstdlib>
#include <list>
#include <algorithm>
#include <functional>
#include <kdl/frames.hpp>
#include <kdl/joint.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <kdl_extensions/treeid_vereshchagin_composable.hpp>
#include <kdl_extensions/functionalcomputation_kdltypes.hpp>






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
    double pointMass = 0.25; //in kg
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
    //twoBranchTree.addSegment(segment5, "L2"); //branches connect at joint 3
    //twoBranchTree.addSegment(segment6, "L5");
    //twoBranchTree.addSegment(segment7, "L6");
    //twoBranchTree.addSegment(segment8, "L6");
    //twoBranchTree.addSegment(segment9, "L8");

    //arm root acceleration
    Vector linearAcc(0.0, -9.8, 0.0); //gravitational acceleration along Y
    Vector angularAcc(0.0, 0.0, 0.0);
    Twist rootAcc(linearAcc, angularAcc);


    std::vector<JointState> jstate;
    jstate.resize(twoBranchTree.getNrOfSegments() + 1);
    jstate[0].q = PI / 3.0;
    jstate[0].qdot = 0.2;
    jstate[1].q = -PI / 3.0;
    jstate[1].qdot = 0.4;
    jstate[2].q = PI / 4.0;
    jstate[2].qdot = -0.2;
    std::vector<SegmentState> lstate;
    lstate.resize(twoBranchTree.getNrOfSegments() + 1);
    std::vector<SegmentState> lstate2;
    lstate2.resize(twoBranchTree.getNrOfSegments() + 1);
    /*
    ForwardKinematics fkcomputation(rootAcc);
    ForceComputer forcecomputer;

    
    //computation for a single node
    lstate[0] = fkcomputation(twoBranchTree.getSegments().begin(), jstate[0]);
    KDL::Wrench force = forcecomputer(twoBranchTree.getSegments().begin(), lstate[0]);
    //std::cout << "value of returned frame" << lstate[0].X << std::endl;

    //computation for the whole tree
    Frame pose;
    //option 1: uses external iterator, user needs to check conditions
    for (SegmentMap::const_iterator iter = twoBranchTree.getSegments().begin(); iter != twoBranchTree.getSegments().end(); iter++)
    {
        lstate[iter->second.q_nr] = fkcomputation(iter, jstate[iter->second.q_nr]);
        force = forcecomputer(iter, lstate[iter->second.q_nr]);
        pose = pose * lstate[iter->second.q_nr].X;

         std::cout << "Loop:: value of returned joint index" << iter->second.q_nr << std::endl;
         std::cout << "Loop:: value of returned local frame" << lstate[iter->second.q_nr].X << std::endl;
        std::cout << "Loop:: value of returned global frame" << pose << std::endl;
        std::cout << "Loop:: value of returned link force" << force << std::endl;
    }
    std::cout << std::endl << std::endl << "Here, loop is done" << std::endl << std::endl << std::endl;

    //option 2: uses internal iterator, inflexible because difficult to define a closure
    transform(twoBranchTree.getSegments().begin(), twoBranchTree.getSegments().end(), jstate.begin(), lstate.begin(), fkcomputation);
     */

    lstate[0].Xdotdot = rootAcc;
    //use case relying in templates
    using namespace kdl_extensions;
    kdl_extensions::transform<tree_iterator, pose> _comp1;
    kdl_extensions::transform<tree_iterator, twist> _comp2;
    kdl_extensions::transform<tree_iterator, accTwist> _comp3;
    kdl_extensions::project<tree_iterator, wrench> _comp4;
   
    std::cout << "Transform initial state" << lstate[0].X << std::endl;
    std::cout << "Twist initial state" << lstate[0].Xdot << std::endl;
    std::cout << "Acc Twist initial state" << lstate[0].Xdotdot << std::endl;
    std::cout << "Wrench initial state" << lstate[0].F << std::endl << std::endl;

    lstate[1] = kdl_extensions::compose(kdl_extensions::compose(_comp4, _comp3), kdl_extensions::compose(_comp2, _comp1))(twoBranchTree.getSegment("L1"), jstate[0], lstate[0]);
    std::cout << "Transform L1" << lstate[1].X << std::endl;
    std::cout << "Twist L1" << lstate[1].Xdot << std::endl;
    std::cout << "Acc Twist L1" << lstate[1].Xdotdot << std::endl;
    std::cout << "Wrench L1" << lstate[1].F << std::endl << std::endl;

    lstate[2] = kdl_extensions::compose(kdl_extensions::compose(_comp4, _comp3), kdl_extensions::compose(_comp2, _comp1))(twoBranchTree.getSegment("L2"), jstate[1], lstate[1]);

    std::cout << "Transform L2" << lstate[2].X << std::endl;
    std::cout << "Twist L2" << lstate[2].Xdot << std::endl;
    std::cout << "Acc Twist L2" << lstate[2].Xdotdot << std::endl;
    std::cout << "Wrench L2" << lstate[2].F << std::endl << std::endl;
   
    typedef Composite<kdl_extensions::transform<tree_iterator, twist>, kdl_extensions::transform<tree_iterator, pose> > compositeType1;
    typedef Composite< kdl_extensions::project<tree_iterator, wrench>, kdl_extensions::transform<tree_iterator, accTwist> > compositeType2;
    typedef Composite<compositeType2, compositeType1> compositeType3;

    compositeType1 composite1 = kdl_extensions::compose(_comp2, _comp1);
    compositeType3 composite2 = kdl_extensions::compose(kdl_extensions::compose(_comp4, _comp3), kdl_extensions::compose(_comp2, _comp1));

    DFSPolicy<KDL::Tree> mypolicy;
    //DFSPolicy<KDL::Chain> mypolicy1;

    std::cout << std::endl << std::endl<< "TRAVERSAL TEST" << std::endl << std::endl;
    traverseGraph(twoBranchTree, composite2, mypolicy)(jstate, lstate, lstate2);
    //traverseGraph(twoBranchTree, kdl_extensions::compose(kdl_extensions::compose(_comp3, _comp2), _comp1), mypolicy)(jstate, lstate, lstate2);
    for(unsigned int i = 0 ; i < twoBranchTree.getNrOfSegments()+1; i++)
    {
        std::cout << std::endl << lstate2[i].X << std::endl;
        std::cout << lstate2[i].Xdot << std::endl;
        std::cout << lstate2[i].Xdotdot << std::endl;
    }
    //This is just used as a reference to compare to our result.
    //using standard KDL forward pose and vel solvers
    /*
    TreeFkSolverPos_recursive kdlPoseSolver(twoBranchTree);
    Frame myPose;
    std::string segmentName = "L2";
    JntArray myJstatePose(twoBranchTree.getNrOfJoints());
    myJstatePose(0) = PI / 3.0;
    myJstatePose(1) = -PI / 3.0;
    myJstatePose(2) = PI / 4.0;

    //this returns the global computational state
    kdlPoseSolver.JntToCart(myJstatePose, myPose, segmentName);
    std::cout << myPose;
    */
    return 0;
}



