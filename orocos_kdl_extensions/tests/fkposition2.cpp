/*
 * File:   poseoperation4treelink-test.cpp
 * Author: azamat
 *
 * Created on Jan 14, 2013, 2:19:59 PM
 */

//#define VERBOSE_CHECK
#define VERBOSE_MAIN

#include <kdl_extensions/functionalcomputation_kdltypes.hpp>
#include <kdl/treefksolverpos_recursive.hpp>

using namespace std;
using namespace KDL;
using namespace kdle;

void createMyTree(KDL::Tree& a_tree)
{
    unsigned int numberofsegments = 90;
    unsigned int numberofbranches = 5;

    std::string linknamecontainer[numberofsegments];
    Joint jointcontainer[numberofsegments];
    Frame framecontainer[numberofsegments];
    Segment segmentcontainer[numberofsegments];
    RigidBodyInertia inertiacontainer[numberofsegments];
    RotationalInertia rotInerSeg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0); //around symmetry axis of rotation
    double pointMass = 0.25; //in kg


    for (unsigned int i = 0; i < numberofsegments - 2; i = i + 3)
    {

        ostringstream converter, converter3;
        converter << "joint" <<i;
        std::string jointname = converter.str();
        converter3 << "link" <<i;
        std::string linkname = converter3.str();
        linknamecontainer[i] = linkname;
//        std::cout << jointname << linkname << std::endl;

        jointcontainer[i] = Joint(jointname, Joint::RotZ, 1, 0, 0.01);
        framecontainer[i] = Frame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
        segmentcontainer[i] = Segment(linkname, jointcontainer[i], framecontainer[i]);
        inertiacontainer[i] = RigidBodyInertia(pointMass, Vector(0.0, -0.4, 0.0), rotInerSeg);
        segmentcontainer[i].setInertia(inertiacontainer[i]);

        ostringstream converter1, converter4;
        converter1 << "joint" << i + 1;
        jointname = converter1.str();
        converter4 << "link" << i + 1;
        linkname = converter4.str();
        linknamecontainer[i+1] = linkname;
//        std::cout << jointname << linkname << std::endl;

        jointcontainer[i + 1] = Joint(jointname, Joint::RotX, 1, 0, 0.01);
        framecontainer[i+1] = Frame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
        segmentcontainer[i+1] = Segment(linkname, jointcontainer[i+1], framecontainer[i+1]);
        inertiacontainer[i+1] = RigidBodyInertia(pointMass, Vector(0.0, -0.4, 0.0), rotInerSeg);
        segmentcontainer[i+1].setInertia(inertiacontainer[i+1]);

        ostringstream converter2, converter5;
        converter2 << "joint" << i + 2;
        jointname = converter2.str();
        converter5 << "link" << i + 2;
        linkname = converter5.str();
        linknamecontainer[i+2] = linkname;
//        std::cout << jointname << linkname << std::endl;

        jointcontainer[i+2] = Joint(jointname, Joint::RotY, 1, 0, 0.01);
        framecontainer[i+2] = Frame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
        segmentcontainer[i+2] = Segment(linkname, jointcontainer[i+2], framecontainer[i+2]);
        inertiacontainer[i+2] = RigidBodyInertia(pointMass, Vector(0.0, -0.4, 0.0), rotInerSeg);
        segmentcontainer[i+2].setInertia(inertiacontainer[i+2]);
    }

    a_tree.addSegment(segmentcontainer[0], "L0");
    
    for (unsigned int i = 0; i < numberofbranches-1; i++)
    {
        a_tree.addSegment(segmentcontainer[i+1], linknamecontainer[i]);
        std::cout << linknamecontainer[i] << " and " << segmentcontainer[i+1].getName() << std::endl;
    }

    
        for (unsigned int j = numberofbranches; j < numberofsegments/numberofbranches; j++)
        {
            a_tree.addSegment(segmentcontainer[j+1], linknamecontainer[j]);
            std::cout << linknamecontainer[j] << " and " << segmentcontainer[j+1].getName() << std::endl;
        }

        for (unsigned int j = numberofsegments/numberofbranches; j < 2*(numberofsegments/numberofbranches); j++)
        {
            a_tree.addSegment(segmentcontainer[j+1], linknamecontainer[j]);
            std::cout << linknamecontainer[j] << " and " << segmentcontainer[j+1].getName() << std::endl;
        }

        for (unsigned int j = 2*(numberofsegments/numberofbranches); j < 3*(numberofsegments/numberofbranches); j++)
        {
            a_tree.addSegment(segmentcontainer[j+1], linknamecontainer[j]);
            std::cout << linknamecontainer[j] << " and " << segmentcontainer[j+1].getName() << std::endl;
        }

        for (unsigned int j = 3*(numberofsegments/numberofbranches); j < 4*(numberofsegments/numberofbranches); j++)
        {
            a_tree.addSegment(segmentcontainer[j+1], linknamecontainer[j]);
            std::cout << linknamecontainer[j] << " and " << segmentcontainer[j+1].getName() << std::endl;
        }

        for (unsigned int j = 4*(numberofsegments/numberofbranches); j < 5*(numberofsegments/numberofbranches)-1; j++)
        {
            a_tree.addSegment(segmentcontainer[j+1], linknamecontainer[j]);
            std::cout << linknamecontainer[j] << " and " << segmentcontainer[j+1].getName() << std::endl;
        }

    


}

int main(int argc, char** argv)
{

    std::cout << "Computing forward position kinematics for a tree" << std::endl;
    Tree twoBranchTree("L0");
    createMyTree(twoBranchTree);

    //    //arm root acceleration
    //    Vector linearAcc(0.0, 0.0, -9.8); //gravitational acceleration along Z
    //    Vector angularAcc(0.0, 0.0, 0.0);
    //    Twist rootAcc(linearAcc, angularAcc);
    //
    //    std::vector<kdle::JointState> jstate;
    //    jstate.resize(twoBranchTree.getNrOfSegments() + 1);
    //    jstate[0].q = PI / 3.0;
    //    jstate[0].qdot = 0.2;
    //    jstate[1].q = -PI / 3.0;
    //    jstate[1].qdot = 0.4;
    //    jstate[2].q = PI / 4.0;
    //    jstate[2].qdot = -0.2;
    //    std::vector<kdle::SegmentState> lstate;
    //    lstate.resize(twoBranchTree.getNrOfSegments() + 1);
    //    printf("Number of Joints %d\n", twoBranchTree.getNrOfJoints());
    //    printf("Number of Segments %d\n", twoBranchTree.getNrOfSegments());
    //
    //    std::vector<kdle::SegmentState> lstate2;
    //    lstate2.resize(twoBranchTree.getNrOfSegments() + 1);
    //    lstate[0].Xdotdot = rootAcc;
    //
    //    // declare a computation to be performed
    //    kdle::transform<tree_iterator, pose> poseComputation;
    //    kdle::accumulate<tree_iterator> poseBaseComputation(lstate[0]);
    //
    //
    //    //declare a policy for a tree traversal
    //    kdle::DFSPolicy_ver2<Tree, outward> forwardTraversal;
    //
    //    //declare a traversal operation on the give topology
    //    traverseGraph_ver2(twoBranchTree, compose(poseBaseComputation, poseComputation), forwardTraversal)(jstate, lstate, lstate2);
    //
    //    //print the results
    //#ifdef VERBOSE_MAIN
    //    for (unsigned int i = 0; i < twoBranchTree.getNrOfSegments(); i++)
    //    {
    //        std::cout << lstate2[i].segmentName << std::endl;
    //        std::cout << std::endl << lstate2[i].X << std::endl;
    //        std::cout << std::endl << lstate2[i].Xtotal << std::endl;
    //    }
    //
    //#endif

    return (EXIT_SUCCESS);
}


