/*
 * File:   poseoperation4treelink-test.cpp
 * Author: azamat
 *
 * Created on Jan 14, 2013, 2:19:59 PM
 */

//#define VERBOSE_CHECK
//#define VERBOSE_MAIN

#include <kdl_extensions/functionalcomputation_kdl.hpp>

using namespace KDL;

void createMyTree(KDL::Tree& twoBranchTree)
{
    Joint joint1 = Joint("j1", Joint::RotZ, 1, 0, 0.01);
    Joint joint2 = Joint("j2", Joint::RotZ, 1, 0, 0.01);
    Joint joint3 = Joint("j3", Joint::RotZ, 1, 0, 0.01);
    Joint joint4 = Joint("j4", Joint::RotZ, 1, 0, 0.01);
    Joint joint5 = Joint("j5", Joint::RotZ, 1, 0, 0.01);
    Joint joint6 = Joint("j6", Joint::RotZ, 1, 0, 0.01);
    Joint joint7 = Joint("j7", Joint::RotZ, 1, 0, 0.01);
    Joint joint8 = Joint("j8", Joint::RotZ, 1, 0, 0.01);
    Joint joint9 = Joint("j9", Joint::RotZ, 1, 0, 0.01);
    Joint joint10 = Joint("j10", Joint::RotZ, 1, 0, 0.01);

    Frame frame1(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
    Frame frame2(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
    Frame frame3(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
    Frame frame4(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
    Frame frame5(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
    Frame frame6(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
    Frame frame7(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
    Frame frame8(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
    Frame frame9(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
    Frame frame10(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));

    Segment segment1 = Segment("L1", joint1, frame1);
    Segment segment2 = Segment("L2", joint2, frame2);
    Segment segment3 = Segment("L3", joint3, frame3);
    Segment segment4 = Segment("L4", joint4, frame4);
    Segment segment5 = Segment("L5", joint5, frame5);
    Segment segment6 = Segment("L6", joint6, frame6);
    Segment segment7 = Segment("L7", joint7, frame7);
    Segment segment8 = Segment("L8", joint8, frame8);
    Segment segment9 = Segment("L9", joint9, frame9);
    Segment segment10 = Segment("M0", joint10, frame10);

    RotationalInertia rotInerSeg1(0.0, 0.0, 0.0, 0.0, 0.0, 0.0); //around symmetry axis of rotation
    double pointMass = 0.25; //in kg
    RigidBodyInertia inerSegment1(pointMass, Vector(0.0, -0.4, 0.0), rotInerSeg1);
    RigidBodyInertia inerSegment2(pointMass, Vector(0.0, -0.4, 0.0), rotInerSeg1);
    RigidBodyInertia inerSegment3(pointMass, Vector(0.0, -0.4, 0.0), rotInerSeg1);
    RigidBodyInertia inerSegment4(pointMass, Vector(0.0, -0.4, 0.0), rotInerSeg1);
    RigidBodyInertia inerSegment5(pointMass, Vector(0.0, -0.4, 0.0), rotInerSeg1);
    RigidBodyInertia inerSegment6(pointMass, Vector(0.0, -0.4, 0.0), rotInerSeg1);
    RigidBodyInertia inerSegment7(pointMass, Vector(0.0, -0.4, 0.0), rotInerSeg1);
    RigidBodyInertia inerSegment8(pointMass, Vector(0.0, -0.4, 0.0), rotInerSeg1);
    RigidBodyInertia inerSegment9(pointMass, Vector(0.0, -0.4, 0.0), rotInerSeg1);
    RigidBodyInertia inerSegment10(pointMass, Vector(0.0, -0.4, 0.0), rotInerSeg1);

    segment1.setInertia(inerSegment1);
    segment2.setInertia(inerSegment2);
    segment3.setInertia(inerSegment3);
    segment4.setInertia(inerSegment4);
    segment5.setInertia(inerSegment5);
    segment6.setInertia(inerSegment6);
    segment7.setInertia(inerSegment7);
    segment8.setInertia(inerSegment8);
    segment9.setInertia(inerSegment9);
    segment10.setInertia(inerSegment10);


    twoBranchTree.addSegment(segment1, "L0");
    twoBranchTree.addSegment(segment2, "L1");
    twoBranchTree.addSegment(segment3, "L2");
    twoBranchTree.addSegment(segment4, "L3");
    twoBranchTree.addSegment(segment10, "L4");
    twoBranchTree.addSegment(segment5, "L2"); //branches connect at joint 3 and j5 is co-located with j3
    twoBranchTree.addSegment(segment6, "L5");
    twoBranchTree.addSegment(segment7, "L6");
    twoBranchTree.addSegment(segment8, "L7");
    twoBranchTree.addSegment(segment9, "L8");

}

int main(int argc, char** argv) {

    std::cout << "Computing forward velocity kinematics for a tree" << std::endl;
    Tree twoBranchTree("L0");
    createMyTree(twoBranchTree);

        //arm root acceleration
    Vector linearAcc(0.0, 0.0, -9.8); //gravitational acceleration along Z
    Vector angularAcc(0.0, 0.0, 0.0);
    Twist rootAcc(linearAcc, angularAcc);

    std::vector<kdle::JointState> jstate;
    jstate.resize(twoBranchTree.getNrOfSegments() + 1);
    jstate[0].q = PI / 3.0;
    jstate[0].qdot = 0.2;
    jstate[1].q = -PI / 3.0;
    jstate[1].qdot = 0.4;
    jstate[2].q = PI / 4.0;
    jstate[2].qdot = -0.2;
    std::vector<kdle::SegmentState> lstate;
    lstate.resize(twoBranchTree.getNrOfSegments() + 1);
    printf("Number of Joints %d\n", twoBranchTree.getNrOfJoints());
    printf("Number of Segments %d\n", twoBranchTree.getNrOfSegments());

    std::vector<kdle::SegmentState> lstate2;
    lstate2.resize(twoBranchTree.getNrOfSegments() + 1);
    lstate[0].Xdotdot = rootAcc;

    //================================Definition of an algorithm=========================//
    // declare a computation to be performed
    kdle::transform<kdle::kdl_tree_iterator, kdle::pose> poseComputation;
    kdle::transform<kdle::kdl_tree_iterator, kdle::twist> twistComputation;
    

    //declare a policy for a tree traversal
    kdle::DFSPolicy<Tree, kdle::outward> forwardTraversal;

    //declare a traversal operation on the give topology
    kdle::traverseGraph(twoBranchTree, kdle::compose(twistComputation, poseComputation), forwardTraversal)(jstate, lstate, lstate2);
    //================================end of the definition===========================//

    //print the results
#ifdef VERBOSE_MAIN
    for (unsigned int i = 0; i < twoBranchTree.getNrOfSegments(); i++)
    {
        std::cout << lstate2[i].segmentName << std::endl;
        std::cout << std::endl << lstate2[i].X << std::endl;
        std::cout << lstate2[i].Xdot << std::endl;
        std::cout << lstate2[i].Xdotdot << std::endl;
        std::cout << lstate2[i].F << std::endl;
    }

#endif


    return (EXIT_SUCCESS);
}

