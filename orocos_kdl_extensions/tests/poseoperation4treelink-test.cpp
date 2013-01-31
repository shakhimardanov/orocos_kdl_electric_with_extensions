/* 
 * File:   poseoperation4treelink-test.cpp
 * Author: azamat
 *
 * Created on Jan 14, 2013, 2:19:59 PM
 */

#include <stdlib.h>
#include <iostream>
#include <kdl_extensions/functionalcomputation_kdltypes.hpp>


/*
 * Simple C++ Test Suite
 */

using namespace KDL;
using namespace kdle;

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


void test1() {
    std::cout << "poseoperation4treelink-test test 1" << std::endl;

    KDL::Tree twoBranchTree("L0");
    createMyTree(twoBranchTree);
    kdle::SegmentState linkState0, linkState1;
    kdle::JointState jointState0;
    kdle::transform<kdle::tree_iterator, kdle::pose> comp1;
    
    linkState1 = comp1(twoBranchTree.getSegment("L1"), jointState0, linkState0);
}

void test2() {
    std::cout << "poseoperation4treelink-test test 2" << std::endl;

    KDL::Tree twoBranchTree("L0");
    createMyTree(twoBranchTree);
    kdle::SegmentState linkState0, linkState1;
    kdle::JointState jointState0;
    kdle::transform<kdle::tree_iterator, kdle::pose> comp1;

    std::cout << "%TEST_FAILED% time=0 testname=test2 (poseoperation4treelink-test) message=error message sample" << std::endl;
    linkState1 = comp1(twoBranchTree.getSegment("L1"), jointState0, linkState0);
    linkState1 = comp1(twoBranchTree.getSegment("L2"), jointState0, linkState1);
}

int main(int argc, char** argv) {
    std::cout << "%SUITE_STARTING% poseoperation4treelink-test" << std::endl;
    std::cout << "%SUITE_STARTED%" << std::endl;

    std::cout << "%TEST_STARTED% test1 (poseoperation4treelink-test)" << std::endl;
    test1();
    std::cout << "%TEST_FINISHED% time=0 test1 (poseoperation4treelink-test)" << std::endl;

    std::cout << "%TEST_STARTED% test2 (poseoperation4treelink-test)\n" << std::endl;
    test2();
    std::cout << "%TEST_FINISHED% time=0 test2 (poseoperation4treelink-test)" << std::endl;

    std::cout << "%SUITE_FINISHED% time=0" << std::endl;

    return (EXIT_SUCCESS);
}

