/*
 * File:   poseoperation4treelink-test.cpp
 * Author: azamat
 *
 * Created on Jan 14, 2013, 2:19:59 PM
 */

//#define VERBOSE_CHECK
#define VERBOSE_MAIN

#include <graphviz/gvc.h>
#include <graphviz/graph.h>
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


    //create names and segments of the tree
    for (unsigned int i = 0; i < numberofsegments - 2; i = i + 3)
    {

        ostringstream converter, converter3;
        converter << "joint" << i;
        std::string jointname = converter.str();
        converter3 << "link" << i;
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
        linknamecontainer[i + 1] = linkname;
        //        std::cout << jointname << linkname << std::endl;

        jointcontainer[i + 1] = Joint(jointname, Joint::RotX, 1, 0, 0.01);
        framecontainer[i + 1] = Frame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
        segmentcontainer[i + 1] = Segment(linkname, jointcontainer[i + 1], framecontainer[i + 1]);
        inertiacontainer[i + 1] = RigidBodyInertia(pointMass, Vector(0.0, -0.4, 0.0), rotInerSeg);
        segmentcontainer[i + 1].setInertia(inertiacontainer[i + 1]);

        ostringstream converter2, converter5;
        converter2 << "joint" << i + 2;
        jointname = converter2.str();
        converter5 << "link" << i + 2;
        linkname = converter5.str();
        linknamecontainer[i + 2] = linkname;
        //        std::cout << jointname << linkname << std::endl;

        jointcontainer[i + 2] = Joint(jointname, Joint::RotY, 1, 0, 0.01);
        framecontainer[i + 2] = Frame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
        segmentcontainer[i + 2] = Segment(linkname, jointcontainer[i + 2], framecontainer[i + 2]);
        inertiacontainer[i + 2] = RigidBodyInertia(pointMass, Vector(0.0, -0.4, 0.0), rotInerSeg);
        segmentcontainer[i + 2].setInertia(inertiacontainer[i + 2]);
    }

    //add created segments to the tree (1 initial base chain + 5 x branches)

    //connect initial base chain to tree root
    a_tree.addSegment(segmentcontainer[0], "L0");

    for (unsigned int i = 0; i < numberofbranches - 1; i++) //chain including link0-link4 (5 segments)
    {
        a_tree.addSegment(segmentcontainer[i + 1], linknamecontainer[i]);
        std::cout << linknamecontainer[i] << " and " << segmentcontainer[i + 1].getName() << std::endl;
    }

    int initialChainElementNumber = a_tree.getNrOfSegments(); //number of segments in initial base chain section of the tree
    int elementsInBranch = (numberofsegments - initialChainElementNumber) / numberofbranches; //number of elements in each branch

    //connect 1st branch to the last link of the initial chain
    a_tree.addSegment(segmentcontainer[numberofbranches], linknamecontainer[numberofbranches - 1]);

//    //segments of the 1st tree branch
//    for (unsigned int j = numberofbranches; j < (elementsInBranch + initialChainElementNumber) - 1; j++)
//    {
//        a_tree.addSegment(segmentcontainer[j + 1], linknamecontainer[j]);
//        std::cout << linknamecontainer[j] << " and " << segmentcontainer[j + 1].getName() << std::endl;
//    }
//
//    //connect 2nd branch to the last link of the initial chain
//    a_tree.addSegment(segmentcontainer[(elementsInBranch + initialChainElementNumber)], linknamecontainer[numberofbranches - 1]);

//    //segments of the 2nd tree branch
//    for (unsigned int j = (elementsInBranch + initialChainElementNumber); j < (2 * elementsInBranch + initialChainElementNumber) - 1; j++)
//    {
//        a_tree.addSegment(segmentcontainer[j + 1], linknamecontainer[j]);
//        std::cout << linknamecontainer[j] << " and " << segmentcontainer[j + 1].getName() << std::endl;
//    }
//
//    //connect 3rd branch to the last link of the initial chain
//    a_tree.addSegment(segmentcontainer[(2 * elementsInBranch + initialChainElementNumber)], linknamecontainer[numberofbranches - 1]);
//
//    //segments of the 3rd tree branch
//    for (unsigned int j = (2 * elementsInBranch + initialChainElementNumber); j < (3 * elementsInBranch + initialChainElementNumber) - 1; j++)
//    {
//        a_tree.addSegment(segmentcontainer[j + 1], linknamecontainer[j]);
//        std::cout << linknamecontainer[j] << " and " << segmentcontainer[j + 1].getName() << std::endl;
//    }
//
//    //connect 4th branch to the last link of the initial chain
//    a_tree.addSegment(segmentcontainer[(3 * elementsInBranch + initialChainElementNumber)], linknamecontainer[numberofbranches - 1]);
//
//    //segments of the 4ht tree branch
//    for (unsigned int j = (3 * elementsInBranch + initialChainElementNumber); j < (4 * elementsInBranch + initialChainElementNumber) - 1; j++)
//    {
//        a_tree.addSegment(segmentcontainer[j + 1], linknamecontainer[j]);
//        std::cout << linknamecontainer[j] << " and " << segmentcontainer[j + 1].getName() << std::endl;
//    }
//
//    //connect 5th branch to the last link of the initial chain
//    a_tree.addSegment(segmentcontainer[(4 * elementsInBranch + initialChainElementNumber)], linknamecontainer[numberofbranches - 1]);
//
//    //segments of the 5th tree branch
//    for (unsigned int j = (4 * elementsInBranch + initialChainElementNumber); j < (5 * elementsInBranch + initialChainElementNumber) - 1; j++)
//    {
//        a_tree.addSegment(segmentcontainer[j + 1], linknamecontainer[j]);
//        std::cout << linknamecontainer[j] << " and " << segmentcontainer[j + 1].getName() << std::endl;
//    }
}

void drawMyTree(KDL::Tree& twoBranchTree)
{

    //graphviz stuff
    /****************************************/
    Agraph_t *g;
    GVC_t *gvc;

    /* set up a graphviz context */
    gvc = gvContext();
    /* Create a simple digraph */
    g = agopen("robot-structure", AGDIGRAPH);

    //create vector to hold nodes
    std::vector<Agnode_t*> nodeVector;
    nodeVector.resize(twoBranchTree.getSegments().size());
    printf("size of segments in tree map %d\n", twoBranchTree.getSegments().size());
    printf("size of segments in tree %d\n", twoBranchTree.getNrOfSegments());

    //create vector to hold edges
    std::vector<Agedge_t*> edgeVector;
    edgeVector.resize(twoBranchTree.getNrOfJoints() + 1);
    int jointIndex = twoBranchTree.getNrOfJoints() + 1;
    printf("size of joint array %d %d\n", jointIndex, twoBranchTree.getNrOfJoints());

    int segmentIndex = 0;
    //    fill in the node vector by iterating over tree segments
    for (SegmentMap::const_iterator iter = twoBranchTree.getSegments().begin(); iter != twoBranchTree.getSegments().end(); ++iter)

    {
        //it would have been very useful if one could access list of joints of a tree
        //list of segments is already possible
        int stringLength = iter->second.segment.getName().size();
        char name[stringLength + 1];
        strcpy(name, iter->second.segment.getName().c_str());
        //q_nr returned is the same value for the root and the its child. this is a bug
        nodeVector[iter->second.q_nr] = agnode(g, name);
        agsafeset(nodeVector[iter->second.q_nr], "color", "red", "");
        agsafeset(nodeVector[iter->second.q_nr], "shape", "box", "");
        std::cout << "index parent " << iter->second.q_nr << std::endl;
        std::cout << "name parent " << iter->second.segment.getName() << std::endl;
        std::cout << "joint name parent " << iter->second.segment.getJoint().getName() << std::endl;
        std::cout << "joint type parent " << iter->second.segment.getJoint().getType() << std::endl;
        //        if (iter->second.segment.getJoint().getType() == Joint::None) //equals to joint type None
        //        {
        //            int stringLength = iter->second.segment.getJoint().getName().size();
        //            char name[stringLength + 1];
        //            strcpy(name, iter->second.segment.getJoint().getName().c_str());
        //            edgeVector[iter->second.q_nr] = agedge(g, nodeVector[iter->second.q_nr], nodeVector[iter->second.q_nr]);
        //            agsafeset(edgeVector[iter->second.q_nr], "label", name, "");
        //        }
        if (segmentIndex < twoBranchTree.getSegments().size())
            segmentIndex++;

    }



    //fill in edge vector by iterating over joints in the tree
    for (SegmentMap::const_iterator iter = twoBranchTree.getSegments().begin(); iter != twoBranchTree.getSegments().end(); ++iter)

    {
        //TODO: Fix node-edge connection relation
        int stringLength = iter->second.segment.getJoint().getName().size();
        std::cout << "Joint name " << iter->second.segment.getJoint().getName() << std::endl;
        char name[stringLength + 1];
        strcpy(name, iter->second.segment.getJoint().getName().c_str());
        for (std::vector<KDL::SegmentMap::const_iterator>::const_iterator childIter = iter->second.children.begin(); childIter != iter->second.children.end(); childIter++)
        {
            edgeVector[iter->second.q_nr] = agedge(g, nodeVector[iter->second.q_nr], nodeVector[(*childIter)->second.q_nr]);
            agsafeset(edgeVector[iter->second.q_nr], "label", name, "");
        }

        //            if (jointIndex != 0)
        //            {
        //                edgeVector[jointIndex] = agedge(g, nodeVector[segmentIndex], nodeVector[jointIndex]);
        //                agsafeset(edgeVector[jointIndex], "label", name, "");
        //            }

    }



    /* Compute a layout using layout engine from command line args */
    //  gvLayoutJobs(gvc, g);
    gvLayout(gvc, g, "dot");

    /* Write the graph according to -T and -o options */
    //gvRenderJobs(gvc, g);
    gvRenderFilename(gvc, g, "ps", "tests.ps");

    /* Free layout data */
    gvFreeLayout(gvc, g);

    /* Free graph structures */
    agclose(g);

    gvFreeContext(gvc);
    /* close output file, free context, and return number of errors */
    return;
}

int main(int argc, char** argv)
{

    std::cout << "Computing forward position kinematics for a tree" << std::endl;
    Tree complexTree("L0");
    createMyTree(complexTree);
    drawMyTree(complexTree);

        //arm root acceleration
        Vector linearAcc(0.0, 0.0, -9.8); //gravitational acceleration along Z
        Vector angularAcc(0.0, 0.0, 0.0);
        Twist rootAcc(linearAcc, angularAcc);
    
        std::vector<kdle::JointState> jstate;
        jstate.resize(complexTree.getNrOfSegments() + 1);
        jstate[0].q = PI / 3.0;
        jstate[0].qdot = 0.2;
        jstate[1].q = -PI / 3.0;
        jstate[1].qdot = 0.4;
        jstate[2].q = PI / 4.0;
        jstate[2].qdot = -0.2;
        jstate[3].q = PI / 4.0;
        jstate[3].qdot = 0.25;
        jstate[4].q = -PI / 8.0;
        jstate[4].qdot = 0.35;

        std::vector<kdle::SegmentState> lstate;
        lstate.resize(complexTree.getNrOfSegments() + 1);
        printf("Number of Joints %d\n", complexTree.getNrOfJoints());
        printf("Number of Segments %d\n", complexTree.getNrOfSegments());
    
        std::vector<kdle::SegmentState> lstate2;
        lstate2.resize(complexTree.getNrOfSegments() + 1);
        lstate[0].Xdotdot = rootAcc;
    
        // declare a computation to be performed
        kdle::transform<tree_iterator, pose> poseComputation;
        kdle::accumulate<tree_iterator> poseBaseComputation(lstate[0]);
    
    
        //declare a policy for a tree traversal
        kdle::DFSPolicy_ver2<Tree, outward> forwardTraversal;
    
        //declare a traversal operation on the give topology
        traverseGraph_ver2(complexTree, compose(poseBaseComputation, poseComputation), forwardTraversal)(jstate, lstate, lstate2);
    
        //print the results
    #ifdef VERBOSE_MAIN
        for (unsigned int i = 0; i < complexTree.getNrOfSegments(); i++)
        {
            std::cout << lstate2[i].segmentName << std::endl;
            std::cout << std::endl << lstate2[i].X << std::endl;
            std::cout << std::endl << lstate2[i].Xtotal << std::endl;
        }
    
    #endif

        TreeFkSolverPos_recursive solverPose(complexTree);
        JntArray q_in(complexTree.getNrOfJoints());
        Frame outputPose;
        
        q_in(0) = jstate[0].q;
        q_in(1) = jstate[1].q;
        q_in(2) = jstate[2].q;
        q_in(3) = jstate[3].q;
        q_in(4) = jstate[4].q;
        
        solverPose.JntToCart(q_in, outputPose, "link5");
        std::cout << outputPose << std::endl;

    return (EXIT_SUCCESS);
}


