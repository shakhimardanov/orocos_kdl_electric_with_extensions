/****************************************************************************** 
*           This file is part of the Geometric harmonization project          *
*                                                                             *
*                            (C) 2014 Azamat Shakhimardanov                   *
*                               Herman Bruyninckx                             *
*                        azamat.shakhimardanov@mech.kuleuven.be               *                              
*                    Department of Mechanical Engineering,                    *
*                   Katholieke Universiteit Leuven, Belgium.                  *
*                                                                             *
*       You may redistribute this software and/or modify it under either the  *
*       terms of the GNU Lesser General Public License version 2.1 (LGPLv2.1  *
*       <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>) or (at your *
*       discretion) of the Modified BSD License:                              *
*       Redistribution and use in source and binary forms, with or without    *
*       modification, are permitted provided that the following conditions    *
*       are met:                                                              *
*       1. Redistributions of source code must retain the above copyright     *
*       notice, this list of conditions and the following disclaimer.         *
*       2. Redistributions in binary form must reproduce the above copyright  *
*       notice, this list of conditions and the following disclaimer in the   *
*       documentation and/or other materials provided with the distribution.  *
*       3. The name of the author may not be used to endorse or promote       *
*       products derived from this software without specific prior written    *
*       permission.                                                           *
*       THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR  *
*       IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED        *
*       WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE    *
*       ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,*
*       INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES    *
*       (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS       *
*       OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) *
*       HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,   *
*       STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING *
*       IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE    *
*       POSSIBILITY OF SUCH DAMAGE.                                           *
*                                                                             *
*******************************************************************************/

//#define VERBOSE_CHECK //switches on console output in kdl related methods
 #define VERBOSE_CHECK_MAIN // switches on console output in main

#include <graphviz/gvc.h>
#include <graphviz/graph.h>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl_extensions/functionalcomputation_kdl.hpp>

using namespace std;


void createMyTree(KDL::Tree& twoBranchTree)
{  
    
    KDL::Joint joint1 = KDL::Joint("j1", KDL::Joint::RotZ, 1, 0, 0.01);
    KDL::Joint joint2 = KDL::Joint("j2", KDL::Joint::RotZ, 1, 0, 0.01);
    KDL::Joint joint3 = KDL::Joint("j3", KDL::Joint::RotZ, 1, 0, 0.01);
    KDL::Joint joint4 = KDL::Joint("j4", KDL::Joint::RotZ, 1, 0, 0.01);
    KDL::Joint joint5 = KDL::Joint("j5", KDL::Joint::RotZ, 1, 0, 0.01);
    KDL::Joint joint6 = KDL::Joint("j6", KDL::Joint::RotZ, 1, 0, 0.01);
    KDL::Joint joint7 = KDL::Joint("j7", KDL::Joint::RotZ, 1, 0, 0.01);
    KDL::Joint joint8 = KDL::Joint("j8", KDL::Joint::RotZ, 1, 0, 0.01);
    KDL::Joint joint9 = KDL::Joint("j9", KDL::Joint::RotZ, 1, 0, 0.01);
    KDL::Joint joint10 = KDL::Joint("j10", KDL::Joint::RotZ, 1, 0, 0.01);

    KDL::Frame frame1(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, -0.4, 0.0));
    KDL::Frame frame2(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, -0.4, 0.0));
    KDL::Frame frame3(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, -0.4, 0.0));
    KDL::Frame frame4(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, -0.4, 0.0));
    KDL::Frame frame5(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, -0.4, 0.0));
    KDL::Frame frame6(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, -0.4, 0.0));
    KDL::Frame frame7(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, -0.4, 0.0));
    KDL::Frame frame8(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, -0.4, 0.0));
    KDL::Frame frame9(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, -0.4, 0.0));
    KDL::Frame frame10(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, -0.4, 0.0));

    KDL::Segment segment1 = KDL::Segment("L1", joint1, frame1);
    KDL::Segment segment2 = KDL::Segment("L2", joint2, frame2);
    KDL::Segment segment3 = KDL::Segment("L3", joint3, frame3);
    KDL::Segment segment4 = KDL::Segment("L4", joint4, frame4);
    KDL::Segment segment5 = KDL::Segment("L5", joint5, frame5);
    KDL::Segment segment6 = KDL::Segment("L6", joint6, frame6);
    KDL::Segment segment7 = KDL::Segment("L7", joint7, frame7);
    KDL::Segment segment8 = KDL::Segment("L8", joint8, frame8);
    KDL::Segment segment9 = KDL::Segment("L9", joint9, frame9);
    KDL::Segment segment10 = KDL::Segment("M0", joint10, frame10);

    KDL::RotationalInertia rotInerSeg1(0.0, 0.0, 0.0, 0.0, 0.0, 0.0); //around symmetry axis of rotation
    double pointMass = 0.25; //in kg
    KDL::RigidBodyInertia inerSegment1(pointMass, KDL::Vector(0.0, -0.4, 0.0), rotInerSeg1);
    KDL::RigidBodyInertia inerSegment2(pointMass, KDL::Vector(0.0, -0.4, 0.0), rotInerSeg1);
    KDL::RigidBodyInertia inerSegment3(pointMass, KDL::Vector(0.0, -0.4, 0.0), rotInerSeg1);
    KDL::RigidBodyInertia inerSegment4(pointMass, KDL::Vector(0.0, -0.4, 0.0), rotInerSeg1);
    KDL::RigidBodyInertia inerSegment5(pointMass, KDL::Vector(0.0, -0.4, 0.0), rotInerSeg1);
    KDL::RigidBodyInertia inerSegment6(pointMass, KDL::Vector(0.0, -0.4, 0.0), rotInerSeg1);
    KDL::RigidBodyInertia inerSegment7(pointMass, KDL::Vector(0.0, -0.4, 0.0), rotInerSeg1);
    KDL::RigidBodyInertia inerSegment8(pointMass, KDL::Vector(0.0, -0.4, 0.0), rotInerSeg1);
    KDL::RigidBodyInertia inerSegment9(pointMass, KDL::Vector(0.0, -0.4, 0.0), rotInerSeg1);
    KDL::RigidBodyInertia inerSegment10(pointMass, KDL::Vector(0.0, -0.4, 0.0), rotInerSeg1);

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
    // twoBranchTree.addSegment(segment5, "L2"); //branches connect at joint 3 and j5 is co-located with j3
    // twoBranchTree.addSegment(segment6, "L5");
    // twoBranchTree.addSegment(segment7, "L6");
    // twoBranchTree.addSegment(segment8, "L7");
    // twoBranchTree.addSegment(segment9, "L8");

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
    for (KDL::SegmentMap::const_iterator iter = twoBranchTree.getSegments().begin(); iter != twoBranchTree.getSegments().end(); ++iter)

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
    for (KDL::SegmentMap::const_iterator iter = twoBranchTree.getSegments().begin(); iter != twoBranchTree.getSegments().end(); ++iter)

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
    gvRenderFilename(gvc, g, "ps", "test-fext.ps");

    /* Free layout data */
    gvFreeLayout(gvc, g);

    /* Free graph structures */
    agclose(g);

    gvFreeContext(gvc);
    /* close output file, free context, and return number of errors */
    return;
}

void computeRNEDynamicsForChain(KDL::Tree& twoBranchTree, const std::string& rootLink, const std::string& tipLink, KDL::Vector& grav,
                                std::vector<kdle::JointState>& jointState, std::vector<kdle::SegmentState>& linkState)
{
    KDL::Chain achain;

    twoBranchTree.getChain(rootLink, tipLink, achain);
    KDL::ChainIdSolver_RNE *rneDynamics = new KDL::ChainIdSolver_RNE(achain, -grav);


    KDL::JntArray q(achain.getNrOfJoints());
    KDL::JntArray q_dot(achain.getNrOfJoints());
    KDL::JntArray q_dotdot(achain.getNrOfJoints());
    KDL::JntArray torques(achain.getNrOfJoints());
    
    KDL::Wrenches f_ext;
    f_ext.resize(achain.getNrOfSegments());
    
    KDL::Vector forceComponent(-2.0, 2.0, 0.0);
//    Vector forceComponent(0.0, 0.0, 0.0);
    KDL::Vector torqueComponent(0.0, 0.0, 0.0);
    KDL::Wrench extForceLastSegment(forceComponent, torqueComponent);

    f_ext[achain.getNrOfSegments()-1] = extForceLastSegment;
    std::cout << endl << endl << endl;
    printf("RNE dynamics values \n");
    std::cout << f_ext[achain.getNrOfSegments()-1] << std::endl;

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

void computeTemplatedDynamicsForTree(KDL::Tree& twoBranchTree, KDL::Vector& grav, std::vector<kdle::JointState>& jointState,
                                     std::vector<kdle::SegmentState>& linkState, std::vector<kdle::SegmentState>& linkState2)
{
    printf("Templated dynamics values for Tree \n");
    kdle::transform<kdle::tree_iterator, kdle::pose> _comp1;
    kdle::transform<kdle::tree_iterator, kdle::twist> _comp2;
    kdle::transform<kdle::tree_iterator, kdle::accTwist> _comp3;
    kdle::balance<kdle::tree_iterator, kdle::force> _comp4;
    kdle::project<kdle::tree_iterator, kdle::wrench> _comp5;
    
#ifdef VERBOSE_CHECK_MAIN
    std::cout << "Transform initial state" << std::endl << linkState[0].X << std::endl;
    std::cout << "Twist initial state" << std::endl << linkState[0].Xdot << std::endl;
    std::cout << "Acc Twist initial state" << std::endl << linkState[0].Xdotdot << std::endl;
    std::cout << "Wrench initial state" << std::endl << linkState[0].F << std::endl << std::endl;
#endif

#ifdef VERBOSE_CHECK_MAIN
    std::cout << "Transform L1" << linkState[1].X << std::endl;
    std::cout << "Twist L1" << linkState[1].Xdot << std::endl;
    std::cout << "Acc Twist L1" << linkState[1].Xdotdot << std::endl;
    std::cout << "Wrench L1" << linkState[1].F << std::endl << std::endl;
#endif

#ifdef VERBOSE_CHECK_MAIN
    std::cout << "Transform L2" << linkState[2].X << std::endl;
    std::cout << "Twist L2" << linkState[2].Xdot << std::endl;
    std::cout << "Acc Twist L2" << linkState[2].Xdotdot << std::endl;
    std::cout << "Wrench L2" << linkState[2].F << std::endl << std::endl;
#endif

    //typedef Composite<kdle::func_ptr(myTestComputation), kdle::func_ptr(myTestComputation) > compositeType0;
    typedef kdle::Composite< kdle::transform<kdle::tree_iterator, kdle::twist>, kdle::transform<kdle::tree_iterator, kdle::pose> > compositeType1;
    typedef kdle::Composite< kdle::balance<kdle::tree_iterator, kdle::force>, kdle::transform<kdle::tree_iterator, kdle::accTwist> > compositeType2;
    typedef kdle::Composite<compositeType2, compositeType1> compositeType3;

//    compositeType1 composite1 = kdle::compose(_comp2, _comp1);
    compositeType3 composite2 = kdle::compose(kdle::compose(_comp4, _comp3), kdle::compose(_comp2, _comp1));

    //kdle::DFSPolicy<KDL::Tree> mypolicy;
    kdle::DFSPolicy_ver2<KDL::Tree, kdle::inward> mypolicy1;
    kdle::DFSPolicy_ver2<KDL::Tree, kdle::outward> mypolicy2;

    std::cout << std::endl << std::endl << "FORWARD TRAVERSAL" << std::endl << std::endl;

//    traverseGraph_ver2(twoBranchTree, composite2, mypolicy2)(jointState, jointState, linkState, linkState2);
     traverseGraph_ver2(twoBranchTree, composite2, mypolicy2)(jointState, linkState, linkState2);

#ifdef VERBOSE_CHECK_MAIN
    std::cout << std::endl << std::endl << "LSTATE" << std::endl << std::endl;
    for (unsigned int i = 0; i < twoBranchTree.getNrOfSegments(); i++)
    {
        std::cout << linkState[i].segmentName << std::endl;
        std::cout << std::endl << linkState[i].X << std::endl;
        std::cout << linkState[i].Xdot << std::endl;
        std::cout << linkState[i].Xdotdot << std::endl;
        std::cout << linkState[i].F << std::endl;
    }
    std::cout << std::endl << std::endl << "LSTATE2" << std::endl << std::endl;
     for (unsigned int i = 0; i < twoBranchTree.getNrOfSegments(); i++)
    {
        std::cout << linkState2[i].segmentName << std::endl;
        std::cout << std::endl << linkState2[i].X << std::endl;
        std::cout << linkState2[i].Xdot << std::endl;
        std::cout << linkState2[i].Xdotdot << std::endl;
        std::cout << linkState2[i].F << std::endl;
    }
#endif

    std::vector<kdle::SegmentState> linkState3;
    linkState3.resize(twoBranchTree.getNrOfSegments()+1);
    std::cout << std::endl << std::endl << "REVERSE TRAVERSAL" << std::endl << std::endl;
    std::vector<kdle::JointState> jstate1;
    jstate1.resize(twoBranchTree.getNrOfSegments() + 1);
    traverseGraph_ver2(twoBranchTree, _comp5, mypolicy1)(jointState, jstate1, linkState2, linkState3);
    //version 1 traversal
    //traverseGraph(twoBranchTree, kdl_extensions::func_ptr(myTestComputation), mypolicy)(1, 2, 3);    
#ifdef VERBOSE_CHECK_MAIN
    std::cout << std::endl << std::endl << "LSTATE3" << std::endl << std::endl;
    for (KDL::SegmentMap::const_reverse_iterator iter = twoBranchTree.getSegments().rbegin(); iter != twoBranchTree.getSegments().rend(); ++iter)
    {
        std::cout << std::endl << iter->first << std::endl;
        std::cout << linkState3[iter->second.q_nr].X << std::endl;
        std::cout << linkState3[iter->second.q_nr].Xdot << std::endl;
        std::cout << linkState3[iter->second.q_nr].Xdotdot << std::endl;
        std::cout << linkState3[iter->second.q_nr].F << std::endl;
        std::cout << "Joint index and torque " << iter->second.q_nr << "  " << jstate1[iter->second.q_nr].torque << std::endl;
    }
#endif
    return;
}

int main(int argc, char** argv)
{

    
    KDL::Tree twoBranchTree("L0");
    createMyTree(twoBranchTree);

    //arm root acceleration
    KDL::Vector linearAcc(0.0, 0.0, -9.8); //gravitational acceleration along Z
    KDL::Vector angularAcc(0.0, 0.0, 0.0);
    KDL::Twist rootAcc(linearAcc, angularAcc);

    std::vector<kdle::JointState> jstate;
    jstate.resize(twoBranchTree.getNrOfSegments() + 1);
    jstate[0].q = KDL::PI / 3.0;
    jstate[0].qdot = 0.2;
    jstate[1].q = -KDL::PI / 3.0;
    jstate[1].qdot = 0.4;
    jstate[2].q = KDL::PI / 4.0;
    jstate[2].qdot = -0.2;

    std::vector<kdle::SegmentState> lstate;
    lstate.resize(twoBranchTree.getNrOfSegments() + 1);
    std::vector<kdle::SegmentState> lstate2;
    lstate2.resize(twoBranchTree.getNrOfSegments() + 1);

    printf("Number of Joints %d\n", twoBranchTree.getNrOfJoints());
    printf("Number of Segments %d\n", twoBranchTree.getNrOfSegments());

    lstate[0].Xdotdot = rootAcc;

    //Add external forces
    int lastSegmentId = twoBranchTree.getNrOfSegments() - 1 ;
    KDL::Vector forceComponent(-2.0, 2.0, 0.0);
//    Vector forceComponent(0.0, 0.0, 0.0);
    KDL::Vector torqueComponent(0.0, 0.0, 0.0);
    KDL::Wrench extForceLastSegment(forceComponent, torqueComponent);

    //external force on the last link
    jstate[lastSegmentId].Fext = extForceLastSegment;
    std::cout << jstate[lastSegmentId].Fext << std::endl;


    computeTemplatedDynamicsForTree(twoBranchTree, linearAcc, jstate, lstate, lstate2);

    std::string rootLink = "L0";
    std::string tipLink = "L9";
    std::vector<kdle::SegmentState> lstate3;
    lstate3.resize(twoBranchTree.getNrOfSegments() + 1);
    std::vector<kdle::SegmentState> lstate4;
    lstate4.resize(twoBranchTree.getNrOfSegments() + 1);
    lstate3[0].Xdotdot = rootAcc;
    

    //This is just used as a reference to compare to our result.
    //using standard KDL forward pose and vel solvers
    rootLink = "L0";
    tipLink = "M0";
    computeRNEDynamicsForChain(twoBranchTree, rootLink, tipLink, linearAcc, jstate, lstate3);

    drawMyTree(twoBranchTree);

    return 0;
}



