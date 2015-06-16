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
#define VERBOSE_WALK 

#include <kdl_extensions/functionalcomputation_kdl.hpp>
#include <kdl_extensions/json_to_semantics_parser.hpp>

using namespace std;

int main(int argc, char** argv)
{
    
    std::string filename("json-models/kinematics/input-kinematics-dsl.json");
    std::string schemaname("/home/azamat/programming/ros-electric/orocos_kinematics_dynamics/orocos_kdl_extensions/json-models/kinematics/kinematics-dsl.json");

//    std::string filename("json-models/geometry/input-geometric-semantics-with-coordinates.json");
//    std::string schemaname("/home/azamat/programming/ros-electric/orocos_kinematics_dynamics/orocos_kdl_extensions/json-models/geometry/geometric-semantics-with-coordinates-dsl.json");
    
    std::vector<SemanticData> semanticData;
    if(!createTree(filename, schemaname, semanticData, true))
    {
        return 1;
    }
    
    unsigned int numberOfSegments = 5;
    unsigned int numberOfJointFramesPerSegment = 3;
    unsigned int numberOfSegmentFrames = 2 * numberOfSegments + 1;
    unsigned int numberOfJointFrames = numberOfJointFramesPerSegment*numberOfSegments;
    
    std::vector< grs::Pose<KDL::Vector, KDL::Rotation> > rootFramePoses;
    std::vector< grs::Pose<KDL::Vector, KDL::Rotation> > tipFramePoses;
    std::vector< grs::Pose<KDL::Vector, KDL::Rotation> > jointFramePoses;

    //SEGMENT METADATA
    //two argument pose
    //LINK1 FRAMES
    // root frame pose wrt base
    KDL::Vector rootFramePositionCoord1 = KDL::Vector(0.0, 0, 0); //position of joint frame's origin
    grs::Position<KDL::Vector> rootFramePosition1(grs::Point("f1.Segment1.Link1"), grs::Body("Segment1.Link1"),
                                                  grs::Point("b.Base"), grs::Body("Base"), grs::OrientationFrame("B.Base"), rootFramePositionCoord1);
    KDL::Rotation rootFrameOrientationCoord1 = KDL::Rotation::RotZ(0.0);
    grs::Orientation<KDL::Rotation> rootFrameOrientation1(grs::OrientationFrame("F1.Segment1.Link1"), grs::Body("Segment1.Link1"),
                                                          grs::OrientationFrame("B.Base"), grs::Body("Base"), grs::OrientationFrame("B.Base"), rootFrameOrientationCoord1);
    grs::Pose<KDL::Vector, KDL::Rotation> rootFramePose1(rootFramePosition1, rootFrameOrientation1);

    //tip frame wrt base
    KDL::Vector tipFramePositionCoord1 = KDL::Vector(0.0, 0.5, 0.0);
    grs::Position<KDL::Vector> tipFramePosition1(grs::Point("l1.Segment1.Link1"), grs::Body("Segment1.Link1"),
                                                 grs::Point("f1.Segment1.Link1"), grs::Body("Segment1.Link1"), grs::OrientationFrame("F1.Segment1.Link1"), tipFramePositionCoord1);
    KDL::Rotation tipFrameOrientationCoord1 = KDL::Rotation::Identity();
    grs::Orientation<KDL::Rotation> tipFrameOrientation1(grs::OrientationFrame("L1.Segment1.Link1"), grs::Body("Segment1.Link1"),
                                                         grs::OrientationFrame("F1.Segment1.Link1"), grs::Body("Segment1.Link1"), grs::OrientationFrame("F1.Segment1.Link1"), tipFrameOrientationCoord1);
    grs::Pose<KDL::Vector, KDL::Rotation> tipFramePose1(tipFramePosition1, tipFrameOrientation1);

    //Link specification with one argument Pose
    kdle::Link< grs::Pose<KDL::Vector, KDL::Rotation> > link1("Link1", rootFramePose1, tipFramePose1);

    //LINK2 FRAMES
    // root frame pose wrt base
    KDL::Vector rootFramePositionCoord2 = KDL::Vector(0.1, 0, 0); //position of joint frame's origin
    grs::Position<KDL::Vector> rootFramePosition2(grs::Point("f2.Segment2.Link2"), grs::Body("Segment2.Link2"),
                                                  grs::Point("b.Base"), grs::Body("Base"), grs::OrientationFrame("B.Base"), rootFramePositionCoord2);
    KDL::Rotation rootFrameOrientationCoord2 = KDL::Rotation::RotZ(0.0);
    grs::Orientation<KDL::Rotation> rootFrameOrientation2(grs::OrientationFrame("F2.Segment2.Link2"), grs::Body("Segment2.Link2"),
                                                          grs::OrientationFrame("B.Base"), grs::Body("Base"), grs::OrientationFrame("B.Base"), rootFrameOrientationCoord2);
    grs::Pose<KDL::Vector, KDL::Rotation> rootFramePose2(rootFramePosition2, rootFrameOrientation2);

    //tip frame wrt base
    KDL::Vector tipFramePositionCoord2 = KDL::Vector(0.5, 0.0, 0.0);
    grs::Position<KDL::Vector> tipFramePosition2(grs::Point("l2.Segment2.Link2"), grs::Body("Segment2.Link2"),
                                                 grs::Point("f2.Segment2.Link2"), grs::Body("Segment2.Link2"), grs::OrientationFrame("F2.Segment2.Link2"), tipFramePositionCoord2);
    KDL::Rotation tipFrameOrientationCoord2 = KDL::Rotation::Identity();
    grs::Orientation<KDL::Rotation> tipFrameOrientation2(grs::OrientationFrame("L2.Segment2.Link2"), grs::Body("Segment2.Link2"),
                                                         grs::OrientationFrame("F2.Segment2.Link2"), grs::Body("Segment2.Link2"), grs::OrientationFrame("F2.Segment2.Link2"), tipFrameOrientationCoord2);
    grs::Pose<KDL::Vector, KDL::Rotation> tipFramePose2(tipFramePosition2, tipFrameOrientation2);

    //Link specification with two argument Pose
    kdle::Link< grs::Pose<KDL::Vector, KDL::Rotation> > link2("Link2", rootFramePose2, tipFramePose2);


    //ATTACHMENT FRAMES OF A SEGMENT

    //LINK1 JOINT FRAMES
    //Joint1
    KDL::Vector link1Joint1FramePositionCoord = KDL::Vector(0.0, 0, 0); //position of joint frame's origin
    grs::Position<KDL::Vector> link1Joint1FramePosition(grs::Point("j1.Segment1.Link1"), grs::Body("Segment1.Link1"),
                                                        grs::Point("f1.Segment1.Link1"), grs::Body("Segment1.Link1"), grs::OrientationFrame("F1.Segment1.Link1"), link1Joint1FramePositionCoord);
    KDL::Rotation link1Joint1FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
    grs::Orientation<KDL::Rotation> link1Joint1FrameOrientation(grs::OrientationFrame("J1.Segment1.Link1"), grs::Body("Segment1.Link1"),
                                                                grs::OrientationFrame("F1.Segment1.Link1"), grs::Body("Segment1.Link1"), grs::OrientationFrame("F1.Segment1.Link1"), link1Joint1FrameOrientationCoord);

    grs::Pose<KDL::Vector, KDL::Rotation> link1Joint1FramePose(link1Joint1FramePosition, link1Joint1FrameOrientation);

    //Joint2
    KDL::Vector link1Joint2FramePositionCoord = KDL::Vector(0, 0.35, 0); //position of joint frame's origin
    grs::Position<KDL::Vector> link1Joint2FramePosition(grs::Point("j2.Segment1.Link1"), grs::Body("Segment1.Link1"),
                                                        grs::Point("f1.Segment1.Link1"), grs::Body("Segment1.Link1"), grs::OrientationFrame("F1.Segment1.Link1"), link1Joint2FramePositionCoord);
    KDL::Rotation link1Joint2FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
    grs::Orientation<KDL::Rotation> link1Joint2FrameOrientation(grs::OrientationFrame("J2.Segment1.Link1"), grs::Body("Segment1.Link1"),
                                                                grs::OrientationFrame("F1.Segment1.Link1"), grs::Body("Segment1.Link1"), grs::OrientationFrame("F1.Segment1.Link1"), link1Joint2FrameOrientationCoord);

    grs::Pose<KDL::Vector, KDL::Rotation> link1Joint2FramePose(link1Joint2FramePosition, link1Joint2FrameOrientation);

    typedef std::vector< kdle::AttachmentFrame<grs::Pose<KDL::Vector, KDL::Rotation> > > AttachmentFrames;
    AttachmentFrames frameList1;

    frameList1.push_back(kdle::createAttachmentFrame(link1Joint1FramePose, kdle::FrameType::JOINT));
    frameList1.push_back(kdle::createAttachmentFrame(link1Joint2FramePose, kdle::FrameType::JOINT));

    //LINK2 JOINT FRAMES
    //Joint1
    KDL::Vector link2Joint1FramePositionCoord = KDL::Vector(0.15, 0, 0); //position of joint frame's origin
    grs::Position<KDL::Vector> link2Joint1FramePosition(grs::Point("j1.Segment2.Link2"), grs::Body("Segment2.Link2"),
                                                        grs::Point("f2.Segment2.Link2"), grs::Body("Segment2.Link2"), grs::OrientationFrame("F2.Segment2.Link2"), link2Joint1FramePositionCoord);
    KDL::Rotation link2Joint1FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
    grs::Orientation<KDL::Rotation> link2Joint1FrameOrientation(grs::OrientationFrame("J1.Segment2.Link2"), grs::Body("Segment2.Link2"),
                                                                grs::OrientationFrame("F2.Segment2.Link2"), grs::Body("Segment2.Link2"), grs::OrientationFrame("F2.Segment2.Link2"), link2Joint1FrameOrientationCoord);
    grs::Pose<KDL::Vector, KDL::Rotation> link2Joint1FramePose(link2Joint1FramePosition, link2Joint1FrameOrientation);

    
    typedef std::vector< kdle::AttachmentFrame<grs::Pose<KDL::Vector, KDL::Rotation> > > AttachmentFrames1;
    AttachmentFrames1 frameList2;

    frameList2.push_back(kdle::createAttachmentFrame(link2Joint1FramePose, kdle::FrameType::JOINT));

    //Segment specification with two argument Pose
    //attaching frames at construction time
    kdle::Segment< grs::Pose<KDL::Vector, KDL::Rotation> > segment1_grs("Segment1", link1, frameList1);
    kdle::Segment< grs::Pose<KDL::Vector, KDL::Rotation> > segment2_grs("Segment2", link2, frameList2);
    
    kdle::JointProperties joint1_props, joint2_props, joint3_props, joint4_props;
    joint1_props = std::make_tuple(kdle::JointTypes::REVOLUTE_X, 0.02, 150.0, -140.0);
    
    kdle::Joint< grs::Pose<KDL::Vector, KDL::Rotation> > joint1("Joint2", segment2_grs.getAttachmentFrames()[0], segment2_grs, segment1_grs.getAttachmentFrames()[1], segment1_grs, joint1_props);

    kdle::TransmissionProperties trans_props = std::make_tuple(0.2, 0.2, 0.2);
    kdle::make_transmission(joint1, trans_props);

    grs::Pose<KDL::Vector, KDL::Rotation> currentJointPose, currentJointPose1, currentJointPose2, currentJointPose3;
    grs::Twist<KDL::Vector, KDL::Vector> currentJointTwist, twisttemp;
    grs::Twist<KDL::Twist> twisttemp1;

    //joint value is of type vector whose size changes according to the joint's DoF
    std::vector<double> jointvalue(1, M_PI / 4.0);
    std::vector<double> jointtwistvalue(1, 0.855);
    std::vector<double> jointtwistvalue1(1, 2.25);
    std::vector<double> jointvalue1(1, -M_PI / 6.0);

    joint1.getPoseOfJointFrames(jointvalue, currentJointPose );
    joint1.getPoseCurrentDistalToPredecessorDistal(jointvalue, currentJointPose1 );
    joint1.getPoseCurrentDistalToPredecessorRefJointFrame(jointvalue, currentJointPose1 );

    joint1.getTwistOfJointFrames(jointtwistvalue, twisttemp);
    joint1.getTwistCurrentDistalToPredecessorJointFrame(jointvalue, jointtwistvalue, twisttemp);

   
    typedef std::vector< kdle::Joint< grs::Pose<KDL::Vector, KDL::Rotation> > > JointList;
    typedef std::map<std::string, kdle::Joint< grs::Pose<KDL::Vector, KDL::Rotation> > > JointListMap;
    JointList jointlist, jointlist1;
    jointlist.push_back(joint1);
   
    kdle::KinematicChain< grs::Pose<KDL::Vector, KDL::Rotation>, JointList > mychain("MyKinematicChain", jointlist);
    std::cout << "mychain.size " << mychain.getNrOfSegments() << std::endl << std::endl << std::endl << std::endl;


    //~SEGMENT METADATA
    //Computational operation
    typedef kdle::Composite< kdle::transform<kdle::grs_iterator, kdle::twist>, kdle::transform<kdle::grs_iterator, kdle::pose> > compositeOperationType;
    kdle::transform<kdle::grs_iterator, kdle::pose> forwardPoseOperation;
    kdle::transform<kdle::grs_iterator, kdle::twist> forwardTwistOperation;
    compositeOperationType forwardKinematics = compose(forwardTwistOperation, forwardPoseOperation);
    //Traversal policy
    kdle::DFSPolicy< kdle::KinematicChain< grs::Pose<KDL::Vector, KDL::Rotation> > > policy;
    std::vector<kdle::ComputationalState<grs::Pose<KDL::Vector, KDL::Rotation>, grs::Twist<KDL::Vector, KDL::Vector>, kdle::StateSpaceType::JointSpace> > jstate;
    jstate.resize(mychain.getNrOfJoints());
    jstate[0].q.push_back(KDL::PI / 4.0);
    jstate[0].qdot.push_back(0.8555);

    std::vector<kdle::ComputationalState<grs::Pose<KDL::Vector, KDL::Rotation>, grs::Twist<KDL::Vector, KDL::Vector>, kdle::StateSpaceType::CartesianSpace> > lstate;
    lstate.resize(mychain.getNrOfJoints());
    std::vector<kdle::ComputationalState<grs::Pose<KDL::Vector, KDL::Rotation>, grs::Twist<KDL::Vector, KDL::Vector>, kdle::StateSpaceType::CartesianSpace> > lstate2;
    lstate2.resize(mychain.getNrOfJoints());
    //Traversal operation
    kdle::traverseGraph(mychain, forwardKinematics, policy)(jstate, lstate, lstate2);

    return 0;
}

