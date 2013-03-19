/*
 * File:   simpleKDLTree.cpp
 * Author: azamat
 *
 * Created on December 21, 2011, 11:46 AM
 */

//#define VERBOSE_CHECK //switches on console output in kdl related methods

//#define VERBOSE_CHECK_MAIN // switches on console output in main


#include <kdl_extensions/functionalcomputation_kdltypes.hpp>

using namespace std;
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
    Segment segment10 = Segment("M0", joint10, frame10);
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

    //Tree twoBranchTree("L0");

    twoBranchTree.addSegment(segment1, "L0");
    twoBranchTree.addSegment(segment2, "L1");
    twoBranchTree.addSegment(segment3, "L2");
    twoBranchTree.addSegment(segment4, "L3");
    twoBranchTree.addSegment(segment10, "L4");
//    twoBranchTree.addSegment(segment5, "L2"); //branches connect at joint 3 and j5 is co-located with j3
//    twoBranchTree.addSegment(segment6, "L5");
//    twoBranchTree.addSegment(segment7, "L6");
//    twoBranchTree.addSegment(segment8, "L7");
//    twoBranchTree.addSegment(segment9, "L8");

}

int main(int argc, char** argv)
{
    std::cout << "Computing inverse dynamics for a tree" << std::endl;

    Tree twoBranchTree("L0");
    createMyTree(twoBranchTree);


    //arm root acceleration
    Vector linearAcc(0.0, 0.0, -9.8); //gravitational acceleration along Z
    Vector angularAcc(0.0, 0.0, 0.0);
    Twist rootAcc(linearAcc, angularAcc);


    std::vector<kdle::JointState> jstate, jstateOut;
    jstate.resize(twoBranchTree.getNrOfSegments() + 1);
    jstateOut.resize(twoBranchTree.getNrOfSegments() + 1);
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
    lstate[0].Xdotdot = rootAcc; //gravitational acceleration along Z

    int lastSegmentId = twoBranchTree.getNrOfSegments() - 1 ;
    Vector forceComponent(-2.0, 2.0, 2.0); 
    Vector torqueComponent(0.0, 0.0, 0.0);
    Wrench extForceLastSegment(forceComponent, torqueComponent);

    lstate[lastSegmentId].Fext = extForceLastSegment;
    std::cout << lstate[lastSegmentId].Fext << std::endl;

//    Initial link computational state
//    std::cout << std::endl << std::endl << "LSTATE0" << std::endl << std::endl;
//    for (unsigned int i = 0; i < twoBranchTree.getNrOfSegments(); i++)
//    {
//        std::cout << lstate[i].segmentName << std::endl;
//        std::cout << std::endl << lstate[i].X << std::endl;
//        std::cout << lstate[i].Xdot << std::endl;
//        std::cout << lstate[i].Xdotdot << std::endl;
//        std::cout << lstate[i].F << std::endl;
//        std::cout << lstate[i].Fext << std::endl;
//    }

    //================================Definition of an algorithm=========================//
    printf("Templated inverse dynamics for Tree \n");
    kdle::transform<tree_iterator, pose> _comp1;
    kdle::transform<tree_iterator, twist> _comp2;
    kdle::transform<tree_iterator, accTwist> _comp3;
    kdle::balance<tree_iterator, force> _comp4;
    kdle::project<tree_iterator, wrench> _comp5;

    //typedef Composite<kdle::func_ptr(myTestComputation), kdle::func_ptr(myTestComputation) > compositeType0;
    typedef Composite< kdle::transform<tree_iterator, twist>, kdle::transform<tree_iterator, pose> > compositeType1;
    typedef Composite< kdle::balance<tree_iterator, force>, kdle::transform<tree_iterator, accTwist> > compositeType2;
    typedef Composite<compositeType2, compositeType1> compositeType3;

    compositeType1 composite1 = kdle::compose(_comp2, _comp1);
    compositeType3 composite2 = kdle::compose(kdle::compose(_comp4, _comp3), kdle::compose(_comp2, _comp1));

    //kdle::DFSPolicy<KDL::Tree> mypolicy;
    kdle::DFSPolicy_ver2<KDL::Tree, inward> mypolicy1;
    kdle::DFSPolicy_ver2<KDL::Tree, outward> mypolicy2;

    std::cout << std::endl << std::endl << "FORWARD TRAVERSAL" << std::endl << std::endl;

//    traverseGraph_ver2(twoBranchTree, composite2, mypolicy2)(jstate, lstate, lstate2); // 3 argument walk takes opers with 4 args
    traverseGraph_ver2(twoBranchTree, composite1, mypolicy2)(jstate, jstateOut, lstate, lstate2); // 4 argument walk takes opers with 5 args
//    traverseGraph_ver2(twoBranchTree, _comp1, mypolicy2)(jstate, jstateOut, lstate, lstate2); // 4 argument walk takes opers with 5 args


#ifdef VERBOSE_CHECK_MAIN
    std::cout << std::endl << std::endl << "LSTATE1" << std::endl << std::endl;
    for (unsigned int i = 0; i < twoBranchTree.getNrOfSegments(); i++)
    {
        std::cout << lstate[i].segmentName << std::endl;
        std::cout << std::endl << lstate[i].X << std::endl;
        std::cout << lstate[i].Xdot << std::endl;
        std::cout << lstate[i].Xdotdot << std::endl;
        std::cout << lstate[i].F << std::endl;
        std::cout << lstate[i].Fext << std::endl;
    }
    std::cout << std::endl << std::endl << "LSTATE2" << std::endl << std::endl;
    for (unsigned int i = 0; i < twoBranchTree.getNrOfSegments(); i++)
    {
        std::cout << lstate2[i].segmentName << std::endl;
        std::cout << std::endl << lstate2[i].X << std::endl;
        std::cout << lstate2[i].Xdot << std::endl;
        std::cout << lstate2[i].Xdotdot << std::endl;
        std::cout << lstate2[i].F << std::endl;
        std::cout << lstate[i].Fext << std::endl;
    }
#endif

    std::vector<kdle::SegmentState> lstate3;
    lstate3.resize(twoBranchTree.getNrOfSegments() + 1);

    std::cout << std::endl << std::endl << "REVERSE TRAVERSAL" << std::endl << std::endl;

    traverseGraph_ver2(twoBranchTree, _comp5, mypolicy1)(jstate, jstateOut, lstate2, lstate3);

    //================================end of the definition===========================//
    
#ifdef VERBOSE_CHECK_MAIN
    std::cout << std::endl << std::endl << "LSTATE3" << std::endl << std::endl;
    for (KDL::SegmentMap::const_reverse_iterator iter = twoBranchTree.getSegments().rbegin(); iter != twoBranchTree.getSegments().rend(); ++iter)
    {
        std::cout << std::endl << iter->first << std::endl;
        std::cout << lstate3[iter->second.q_nr].X << std::endl;
        std::cout << lstate3[iter->second.q_nr].Xdot << std::endl;
        std::cout << lstate3[iter->second.q_nr].Xdotdot << std::endl;
        std::cout << lstate3[iter->second.q_nr].F << std::endl;
        std::cout << lstate3[iter->second.q_nr].Fext << std::endl;
        std::cout << "Joint index and torque " << iter->second.q_nr << "  " << jstateOut[iter->second.q_nr].torque << std::endl;
    }
#endif


    return 0;
}




