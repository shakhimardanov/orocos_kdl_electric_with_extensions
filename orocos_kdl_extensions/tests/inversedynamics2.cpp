/*
 * File:   simpleKDLTree.cpp
 * Author: azamat
 *
 * Created on December 21, 2011, 11:46 AM
 */

//#define VERBOSE_CHECK //switches on console output in kdl related methods

#define VERBOSE_CHECK_MAIN // switches on console output in main
//#define VERBOSE_CHECK // switches on console output in main

#include <graphviz/gvc.h>
#include <graphviz/graph.h>
#include <kdl_extensions/functionalcomputation_kdl.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

using namespace std;
using namespace KDL;


void createMyTree(KDL::Tree& a_tree)
{
    unsigned int numberofsegments = 66;
    unsigned int numberofbranches = 5;

    std::string linknamecontainer[numberofsegments];
    Joint jointcontainer[numberofsegments];
    Frame framecontainer[numberofsegments];
    Segment segmentcontainer[numberofsegments];
    RigidBodyInertia inertiacontainer[numberofsegments];
    RotationalInertia rotInerSeg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0); //around symmetry axis of rotation
    double pointMass = 0.25; //in kg

    std::string jointname, linkname;
    //create names and segments of the tree
    for (unsigned int i = 0; i < numberofsegments - 2; i = i + 3)
    {
        //this name manipulation is required to ensure that
        //topological order of segments in the tree and the order std::map data structure are equivalent
        if (i < 10)
        {
            ostringstream converter, converter3;
            converter << "joint00" << i;
            jointname = converter.str();
            converter3 << "link00" << i;
            linkname = converter3.str();
            linknamecontainer[i] = linkname;
//            std::cout << jointname << linkname << std::endl;
        }
        else
        {
            ostringstream converter, converter3;
            converter << "joint0" << i;
            jointname = converter.str();
            converter3 << "link0" << i;
            linkname = converter3.str();
            linknamecontainer[i] = linkname;
//            std::cout << jointname << linkname << std::endl;

        }
        jointcontainer[i] = Joint(jointname, Joint::RotZ, 1, 0, 0.01);
        framecontainer[i] = Frame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
        segmentcontainer[i] = Segment(linkname, jointcontainer[i], framecontainer[i]);
        inertiacontainer[i] = RigidBodyInertia(pointMass, Vector(0.0, -0.4, 0.0), rotInerSeg);
        segmentcontainer[i].setInertia(inertiacontainer[i]);

        if (i < 9)
        {
            ostringstream converter1, converter4;
            converter1 << "joint00" << i + 1;
            jointname = converter1.str();
            converter4 << "link00" << i + 1;
            linkname = converter4.str();
            linknamecontainer[i + 1] = linkname;
//            std::cout << jointname << linkname << std::endl;
        }
        else
        {
            ostringstream converter1, converter4;
            converter1 << "joint0" << i + 1;
            jointname = converter1.str();
            converter4 << "link0" << i + 1;
            linkname = converter4.str();
            linknamecontainer[i + 1] = linkname;
//            std::cout << jointname << linkname << std::endl;

        }

        jointcontainer[i + 1] = Joint(jointname, Joint::RotX, 1, 0, 0.01);
        framecontainer[i + 1] = Frame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
        segmentcontainer[i + 1] = Segment(linkname, jointcontainer[i + 1], framecontainer[i + 1]);
        inertiacontainer[i + 1] = RigidBodyInertia(pointMass, Vector(0.0, -0.4, 0.0), rotInerSeg);
        segmentcontainer[i + 1].setInertia(inertiacontainer[i + 1]);

        if (i < 8)
        {
            ostringstream converter2, converter5;
            converter2 << "joint00" << i + 2;
            jointname = converter2.str();
            converter5 << "link00" << i + 2;
            linkname = converter5.str();
            linknamecontainer[i + 2] = linkname;
//            std::cout << jointname << linkname << std::endl;
        }

        else
        {
            ostringstream converter2, converter5;
            converter2 << "joint0" << i + 2;
            jointname = converter2.str();
            converter5 << "link0" << i + 2;
            linkname = converter5.str();
            linknamecontainer[i + 2] = linkname;
//            std::cout << jointname << linkname << std::endl;

        }
        jointcontainer[i + 2] = Joint(jointname, Joint::RotY, 1, 0, 0.01);
        framecontainer[i + 2] = Frame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
        segmentcontainer[i + 2] = Segment(linkname, jointcontainer[i + 2], framecontainer[i + 2]);
        inertiacontainer[i + 2] = RigidBodyInertia(pointMass, Vector(0.0, -0.4, 0.0), rotInerSeg);
        segmentcontainer[i + 2].setInertia(inertiacontainer[i + 2]);
    }

    //add created segments to the tree (1 initial base chain + 5 x branches)

    //connect initial base chain to tree root
    a_tree.addSegment(segmentcontainer[0], "L0");
    std::cout << "Initial base chain" << std::endl;
    for (unsigned int i = 0; i < numberofbranches - 1; i++) //chain including link0-link4 (5 segments)
    {
        a_tree.addSegment(segmentcontainer[i + 1], linknamecontainer[i]);
        std::cout << linknamecontainer[i] << " and " << segmentcontainer[i + 1].getName() << std::endl;
    }

    int initialChainElementNumber = a_tree.getNrOfSegments(); //number of segments in initial base chain section of the tree
    int elementsInBranch = (numberofsegments - initialChainElementNumber) / numberofbranches; //number of elements in each branch

    //connect 1st branch to the last link of the initial chain
    a_tree.addSegment(segmentcontainer[numberofbranches], linknamecontainer[numberofbranches - 1]);

    std::cout << "Branch " << numberofbranches-4 << std::endl;
    //segments of the 1st tree branch
    for (unsigned int j = numberofbranches; j < (elementsInBranch + initialChainElementNumber) - 1; j++)
    {
        a_tree.addSegment(segmentcontainer[j + 1], linknamecontainer[j]);
        std::cout << linknamecontainer[j] << " and " << segmentcontainer[j + 1].getName() << std::endl;
    }

    //connect 2nd branch to the last link of the initial chain
    a_tree.addSegment(segmentcontainer[(elementsInBranch + initialChainElementNumber)], linknamecontainer[numberofbranches - 2]);
    
    std::cout << "Branch " << numberofbranches-3 << std::endl;
    //segments of the 2nd tree branch
    for (unsigned int j = (elementsInBranch + initialChainElementNumber); j < (2 * elementsInBranch + initialChainElementNumber) - 1; j++)
    {
        a_tree.addSegment(segmentcontainer[j + 1], linknamecontainer[j]);
        std::cout << linknamecontainer[j] << " and " << segmentcontainer[j + 1].getName() << std::endl;
    }

    //connect 3rd branch to the last link of the initial chain
    a_tree.addSegment(segmentcontainer[(2 * elementsInBranch + initialChainElementNumber)], linknamecontainer[numberofbranches - 3]);

    std::cout << "Branch " << numberofbranches-2 << std::endl;
    //segments of the 3rd tree branch
    for (unsigned int j = (2 * elementsInBranch + initialChainElementNumber); j < (3 * elementsInBranch + initialChainElementNumber) - 1; j++)
    {
        a_tree.addSegment(segmentcontainer[j + 1], linknamecontainer[j]);
        std::cout << linknamecontainer[j] << " and " << segmentcontainer[j + 1].getName() << std::endl;
    }

    //connect 4th branch to the last link of the initial chain
    a_tree.addSegment(segmentcontainer[(3 * elementsInBranch + initialChainElementNumber)], linknamecontainer[numberofbranches - 4]);

    std::cout << "Branch " << numberofbranches-1 << std::endl;
    //segments of the 4ht tree branch
    for (unsigned int j = (3 * elementsInBranch + initialChainElementNumber); j < (4 * elementsInBranch + initialChainElementNumber) - 1; j++)
    {
        a_tree.addSegment(segmentcontainer[j + 1], linknamecontainer[j]);
        std::cout << linknamecontainer[j] << " and " << segmentcontainer[j + 1].getName() << std::endl;
    }

    //connect 5th branch to the last link of the initial chain
    a_tree.addSegment(segmentcontainer[(4 * elementsInBranch + initialChainElementNumber)], linknamecontainer[numberofbranches - 1]);

    //segments of the 5th tree branch
    std::cout << "Branch " << numberofbranches << std::endl;
    for (unsigned int j = (4 * elementsInBranch + initialChainElementNumber); j < (5 * elementsInBranch + initialChainElementNumber) - 1; j++)
    {
        a_tree.addSegment(segmentcontainer[j + 1], linknamecontainer[j]);
        std::cout << linknamecontainer[j] << " and " << segmentcontainer[j + 1].getName() << std::endl;
    }
}

void drawMyTree(KDL::Tree& a_tree)
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
    nodeVector.resize(a_tree.getSegments().size());
//    printf("size of segments in tree map %d\n", a_tree.getSegments().size());
//    printf("size of segments in tree %d\n", a_tree.getNrOfSegments());

    //create vector to hold edges
    std::vector<Agedge_t*> edgeVector;
    edgeVector.resize(a_tree.getNrOfJoints() + 1);
    int jointIndex = a_tree.getNrOfJoints() + 1;
//    printf("size of joint array %d %d\n", jointIndex, a_tree.getNrOfJoints());

    int segmentIndex = 0;
    //    fill in the node vector by iterating over tree segments
    for (SegmentMap::const_iterator iter = a_tree.getSegments().begin(); iter != a_tree.getSegments().end(); ++iter)

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
//                    char name[stringLength + 1];
        //            strcpy(name, iter->second.segment.getJoint().getName().c_str());
        //            edgeVector[iter->second.q_nr] = agedge(g, nodeVector[iter->second.q_nr], nodeVector[iter->second.q_nr]);
        //            agsafeset(edgeVector[iter->second.q_nr], "label", name, "");
        //        }
        if (segmentIndex < a_tree.getSegments().size())
            segmentIndex++;

    }



    //fill in edge vector by iterating over joints in the tree
    for (SegmentMap::const_iterator iter = a_tree.getSegments().begin(); iter != a_tree.getSegments().end(); ++iter)

    {
        //TODO: Fix node-edge connection relation
        int stringLength = iter->second.segment.getJoint().getName().size();
//        std::cout << "Joint name " << iter->second.segment.getJoint().getName() << std::endl;
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
    gvRenderFilename(gvc, g, "ps", "test-invdynamics2.ps");

    /* Free layout data */
    gvFreeLayout(gvc, g);

    /* Free graph structures */
    agclose(g);

    gvFreeContext(gvc);
    /* close output file, free context, and return number of errors */
    return;
}

void computeRNEDynamicsForChain(KDL::Tree& a_tree, const std::string& rootLink, const std::string& tipLink, KDL::Vector& grav,
                                std::vector<kdle::JointState>& jointState, std::vector<kdle::SegmentState>& linkState)
{
    KDL::Chain achain;
    a_tree.getChain(rootLink, tipLink, achain);

    KDL::JntArray q(achain.getNrOfJoints());
    KDL::JntArray q_dot(achain.getNrOfJoints());
    KDL::JntArray q_dotdot(achain.getNrOfJoints());
    JntArray torques(achain.getNrOfJoints());
    KDL::Wrenches f_ext;
    f_ext.resize(achain.getNrOfSegments());

    std::cout << endl << endl;
    printf("RNE dynamics values \n");

    KDL::ChainIdSolver_RNE *rneDynamics = new ChainIdSolver_RNE(achain, -grav);
    
    for (unsigned int i = 0; i < achain.getNrOfJoints(); ++i)
    {
        q(i) = jointState[i].q;
        q_dot(i) = jointState[i].qdot;
        q_dotdot(i) = jointState[i].qdotdot;
        printf("q, qdot %f, %f\n", q(i), q_dot(i));
    }

    
    rneDynamics->CartToJnt(q, q_dot, q_dotdot, f_ext, torques);

    for (unsigned int i = 0; i < achain.getNrOfJoints(); ++i)
    {
        printf("index, q, qdot, torques %d, %f, %f, %f\n", i, q(i), q_dot(i), torques(i));
    }
    return;
}

int main(int argc, char** argv)
{
    std::cout << "Computing inverse dynamics for a tree" << std::endl;

    Tree complexTree("L0");
    createMyTree(complexTree);
    drawMyTree(complexTree);

    //arm root acceleration
    Vector linearAcc(0.0, 0.0, -9.8); //gravitational acceleration along Z
    Vector angularAcc(0.0, 0.0, 0.0);
    Twist rootAcc(linearAcc, angularAcc);

    printf("Number of Joints %d\n", complexTree.getNrOfJoints());
    printf("Number of Segments %d\n", complexTree.getNrOfSegments());

    std::vector<kdle::JointState> jstate;
    jstate.resize(complexTree.getNrOfSegments() + 1);

    for (unsigned int i = 0; i < complexTree.getSegments().size() - 2; i = i + 3)
    {
        jstate[i].q = PI / 3.0;
        jstate[i].qdot = 0.2;
        jstate[i + 1].q = -PI / 3.0;
        jstate[i + 1].qdot = 0.4;
        jstate[i + 2].q = PI / 4.0;
        jstate[i + 2].qdot = -0.2;

    }

    std::vector<kdle::SegmentState> lstate, lstate2;
    lstate.resize(complexTree.getNrOfSegments() + 1);
    lstate2.resize(complexTree.getNrOfSegments() + 1);
    lstate[0].Xdotdot = rootAcc;

    //================================Definition of an algorithm=========================//
    // declare a computation to be performed
    kdle::transform<kdle::tree_iterator, kdle::pose> _comp1;
    kdle::transform<kdle::tree_iterator, kdle::twist> _comp2;
    kdle::transform<kdle::tree_iterator, kdle::accTwist> _comp3;
    kdle::balance<kdle::tree_iterator, kdle::force> _comp4;
    kdle::project<kdle::tree_iterator, kdle::wrench> _comp5;

    //some typedef's to simplify writing of composition. This is not necessary but greatly improve code readability
    typedef kdle::Composite< kdle::transform<kdle::tree_iterator, kdle::twist>, kdle::transform<kdle::tree_iterator, kdle::pose> > compositeType1;
    typedef kdle::Composite< kdle::balance<kdle::tree_iterator, kdle::force>, kdle::transform<kdle::tree_iterator, kdle::accTwist> > compositeType2;
    typedef kdle::Composite<compositeType2, compositeType1> compositeType3;

    //let the operations be composed into a complex operation.
    compositeType1 composite1 = kdle::compose(_comp2, _comp1);
    compositeType3 composite2 = kdle::compose(kdle::compose(_comp4, _comp3), kdle::compose(_comp2, _comp1));

    //declare policies for the tree traversal
    //for inverse dynamics we require two traversal outward and inward
    kdle::DFSPolicy<KDL::Tree, kdle::outward> outwardPolicy;
    kdle::DFSPolicy<KDL::Tree, kdle::inward> inwardPolicy;

    std::cout << std::endl<< "FIRST FORWARD/OUTWARD TRAVERSAL" << std::endl;
    //declare first traversal operation on the given topology
    kdle::traverseGraph(complexTree, composite2, outwardPolicy)(jstate, lstate, lstate2);
    
#ifdef VERBOSE_CHECK_MAIN
//    std::cout << std::endl << std::endl << "LSTATE" << std::endl << std::endl;
//    for (unsigned int i = 0; i < complexTree.getNrOfSegments(); i++)
//    {
//        std::cout << lstate[i].segmentName << std::endl;
//        std::cout << std::endl << lstate[i].X << std::endl;
//        std::cout << lstate[i].Xdot << std::endl;
//        std::cout << lstate[i].Xdotdot << std::endl;
//        std::cout << lstate[i].F << std::endl;
//    }
    std::cout << std::endl << std::endl << "LSTATE2" << std::endl << std::endl;
    for (unsigned int i = 0; i < complexTree.getNrOfSegments(); i++)
    {
        std::cout << lstate2[i].segmentName << std::endl;
        std::cout << std::endl << lstate2[i].X << std::endl;
        std::cout << lstate2[i].Xdot << std::endl;
        std::cout << lstate2[i].Xdotdot << std::endl;
        std::cout << lstate2[i].F << std::endl;
    }
#endif

    std::vector<kdle::SegmentState> lstate3;
    lstate3.resize(complexTree.getNrOfSegments() + 1);

    std::cout << std::endl << std::endl << "FIRST REVERSE/INVERSE TRAVERSAL" << std::endl << std::endl;
    //declare second traversal operation on the given topology
    kdle::traverseGraph(complexTree, _comp5, inwardPolicy)(jstate, jstate, lstate2, lstate3);

    //=============================end of the definition============================//

#ifdef VERBOSE_CHECK_MAIN
    std::cout << std::endl << std::endl << "LSTATE3" << std::endl << std::endl;
    for (KDL::SegmentMap::const_reverse_iterator iter = complexTree.getSegments().rbegin(); iter != complexTree.getSegments().rend(); ++iter)
    {
        std::cout << std::endl << iter->first << std::endl;
        std::cout << lstate3[iter->second.q_nr].X << std::endl;
        std::cout << lstate3[iter->second.q_nr].Xdot << std::endl;
        std::cout << lstate3[iter->second.q_nr].Xdotdot << std::endl;
        std::cout << lstate3[iter->second.q_nr].F << std::endl;
        std::cout << "Joint index and torque " << iter->second.q_nr << "  " << jstate[iter->second.q_nr].torque << std::endl;
    }
#endif


    //compute inverse dynamics for the section of the tree using kdl::rnedynamics
    std::string rootLink = "L0";
    std::string tipLink = "link005";
    std::vector<kdle::SegmentState> lstate4;
    lstate4.resize(complexTree.getNrOfSegments() + 1);
    lstate3[0].Xdotdot = rootAcc;
    
    //This is just used as a reference to compare to our result.
    //using standard KDL forward pose and vel solvers
    computeRNEDynamicsForChain(complexTree, rootLink, tipLink, linearAcc, jstate, lstate3);

    return 0;
}





