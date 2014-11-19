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
#include <kdl_extensions/chain_geometric_primitives.hpp>

using namespace std;

/*
 * 
 */
int main(int argc, char** argv)
{
    //SEGMENT METADATA
        //one argument pose
        //LINK1 FRAMES
        // root frame pose wrt base
        KDL::Vector rootFramePosition1 = KDL::Vector(0.0,0,0); //position of joint frame's origin
        KDL::Rotation rootFrameOrientation1 = KDL::Rotation::RotZ(0.0);
        grs::PoseCoordinates<KDL::Frame> rootFramePoseCoord1(KDL::Frame(rootFrameOrientation1, rootFramePosition1));
        grs::PoseCoordinatesSemantics rootFramePoseSemantics1(grs::Point("f1.Segment1.Link1"),grs::OrientationFrame("F1.Segment1.Link1"),grs::Body("Segment1.Link1"),
                                                              grs::Point("b.Base"),grs::OrientationFrame("B.Base"),grs::Body("Base"),grs::OrientationFrame("B.Base"));
        grs::Pose<KDL::Frame> rootFramePose1(rootFramePoseSemantics1, rootFramePoseCoord1);
        
        //tip frame wrt base
        KDL::Vector tipFramePosition1 = KDL::Vector(0.5, 0.0, 0.0);
        KDL::Rotation tipFrameOrientation1 = KDL::Rotation::Identity();
        grs::PoseCoordinates<KDL::Frame> tipFramePoseCoord1(KDL::Frame(tipFrameOrientation1, tipFramePosition1));
        grs::PoseCoordinatesSemantics tipFramePoseSemantics1(grs::Point("l1.Segment1.Link1"),grs::OrientationFrame("L1.Segment1.Link1"),grs::Body("Segment1.Link1"),
                                                             grs::Point("f1.Segment1.Link1"),grs::OrientationFrame("F1.Segment1.Link1"),grs::Body("Segment1.Link1"),
                                                             grs::OrientationFrame("F1.Segment1.Link1"));
        grs::Pose<KDL::Frame> tipFramePose1(tipFramePoseSemantics1, tipFramePoseCoord1);
        
        //Link specification with one argument Pose
        kdle::Link< grs::Pose<KDL::Frame> > link1("Link1", rootFramePose1, tipFramePose1);
        
        //two argument pose
        //LINK2 FRAMES
        // root frame pose wrt base
        KDL::Vector rootFramePositionCoord2 = KDL::Vector(0.0,0,0); //position of joint frame's origin
        grs::Position<KDL::Vector> rootFramePosition2(grs::Point("f2.Segment2.Link2"),grs::Body("Segment2.Link2"),
                                                      grs::Point("b.Base"), grs::Body("Base"), grs::OrientationFrame("B.Base"), rootFramePositionCoord2);
        KDL::Rotation rootFrameOrientationCoord2 = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> rootFrameOrientation2(grs::OrientationFrame("F2.Segment2.Link2"),grs::Body("Segment2.Link2"), 
                                                              grs::OrientationFrame("B.Base"),grs::Body("Base"), grs::OrientationFrame("B.Base"), rootFrameOrientationCoord2);
        grs::Pose<KDL::Vector, KDL::Rotation> rootFramePose2(rootFramePosition2, rootFrameOrientation2);
        
        //tip frame wrt base
        KDL::Vector tipFramePositionCoord2 = KDL::Vector(0.5, 0.0, 0.0);
        grs::Position<KDL::Vector> tipFramePosition2(grs::Point("l2.Segment2.Link2"),grs::Body("Segment2.Link2"),
                                                     grs::Point("f2.Segment2.Link2"),grs::Body("Segment2.Link2"), grs::OrientationFrame("F2.Segment2.Link2"), tipFramePositionCoord2);
        KDL::Rotation tipFrameOrientationCoord2 = KDL::Rotation::Identity();
        grs::Orientation<KDL::Rotation> tipFrameOrientation2(grs::OrientationFrame("L2.Segment2.Link2"),grs::Body("Segment2.Link2"),
                                                             grs::OrientationFrame("F2.Segment2.Link2"),grs::Body("Segment2.Link2"), grs::OrientationFrame("F2.Segment2.Link2"), tipFrameOrientationCoord2);
        grs::Pose<KDL::Vector, KDL::Rotation> tipFramePose2(tipFramePosition2, tipFrameOrientation2);
        
        //Link specification with two argument Pose
        kdle::Link< grs::Pose<KDL::Vector, KDL::Rotation> > link2("Link2", rootFramePose2, tipFramePose2);
        
        //LINK3 FRAMES
        // root frame pose wrt base
        KDL::Vector rootFramePositionCoord3 = KDL::Vector(0,0,0); //position of joint frame's origin
        grs::Position<KDL::Vector> rootFramePosition3(grs::Point("f3.Segment3.Link3"),grs::Body("Segment3.Link3"),
                                                      grs::Point("b.Base"), grs::Body("Base"), grs::OrientationFrame("B.Base"), rootFramePositionCoord3);
        KDL::Rotation rootFrameOrientationCoord3 = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> rootFrameOrientation3(grs::OrientationFrame("F3.Segment3.Link3"),grs::Body("Segment3.Link3"),
                                                              grs::OrientationFrame("B.Base"),grs::Body("Base"), grs::OrientationFrame("B.Base"), rootFrameOrientationCoord3);
        grs::Pose<KDL::Vector, KDL::Rotation> rootFramePose3(rootFramePosition3, rootFrameOrientation3);
        
        //tip frame wrt base
        KDL::Vector tipFramePositionCoord3 = KDL::Vector(0.5, 0.0, 0.0);
        grs::Position<KDL::Vector> tipFramePosition3(grs::Point("l3.Segment3.Link3"),grs::Body("Segment3.Link3"),
                                                     grs::Point("f3.Segment3.Link3"),grs::Body("Segment3.Link3"), grs::OrientationFrame("F3.Segment3.Link3"), tipFramePositionCoord3);
        KDL::Rotation tipFrameOrientationCoord3 = KDL::Rotation::Identity();
        grs::Orientation<KDL::Rotation> tipFrameOrientation3(grs::OrientationFrame("L3.Segment3.Link3"),grs::Body("Segment3.Link3"),
                                                             grs::OrientationFrame("F3.Segment3.Link3"),grs::Body("Segment3.Link3"), grs::OrientationFrame("F3.Segment3.Link3"), tipFrameOrientationCoord3);
        grs::Pose<KDL::Vector, KDL::Rotation> tipFramePose3(tipFramePosition3, tipFrameOrientation3);
        
        //Link specification with two argument Pose
        kdle::Link< grs::Pose<KDL::Vector, KDL::Rotation> > link3("Link3", rootFramePose3, tipFramePose3);
        
        //LINK4 FRAMES
        // root frame pose wrt base
        KDL::Vector rootFramePositionCoord4 = KDL::Vector(0,0,0); //position of joint frame's origin
        grs::Position<KDL::Vector> rootFramePosition4(grs::Point("f4.Segment4.Link4"),grs::Body("Segment4.Link4"),
                                                      grs::Point("b.Base"), grs::Body("Base"), grs::OrientationFrame("B.Base"), rootFramePositionCoord4);
        KDL::Rotation rootFrameOrientationCoord4 = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> rootFrameOrientation4(grs::OrientationFrame("F4.Segment4.Link4"),grs::Body("Segment4.Link4"),
                                                              grs::OrientationFrame("B.Base"),grs::Body("Base"), grs::OrientationFrame("B.Base"), rootFrameOrientationCoord4);
        grs::Pose<KDL::Vector, KDL::Rotation> rootFramePose4(rootFramePosition4, rootFrameOrientation4);
        
        //tip frame wrt base
        KDL::Vector tipFramePositionCoord4 = KDL::Vector(0.5, 0.0, 0.0);
        grs::Position<KDL::Vector> tipFramePosition4(grs::Point("l4.Segment4.Link4"),grs::Body("Segment4.Link4"),
                                                     grs::Point("f4.Segment4.Link4"),grs::Body("Segment4.Link4"), grs::OrientationFrame("F4.Segment4.Link4"), tipFramePositionCoord4);
        KDL::Rotation tipFrameOrientationCoord4 = KDL::Rotation::Identity();
        grs::Orientation<KDL::Rotation> tipFrameOrientation4(grs::OrientationFrame("L4.Segment4.Link4"),grs::Body("Segment4.Link4"),
                                                             grs::OrientationFrame("F4.Segment4.Link4"),grs::Body("Segment4.Link4"),grs::OrientationFrame("F4.Segment4.Link4"), tipFrameOrientationCoord4);
        grs::Pose<KDL::Vector, KDL::Rotation> tipFramePose4(tipFramePosition4, tipFrameOrientation4);
        
        //Link specification with two argument Pose
        kdle::Link< grs::Pose<KDL::Vector, KDL::Rotation> > link4("Link4", rootFramePose4, tipFramePose4);
        
        //ATTACHMENT FRAMES OF A SEGMENT
        
        //LINK1 JOINT FRAMES
        //Joint1
        KDL::Vector link1Joint1FramePositionCoord = KDL::Vector(0.0,0,0); //position of joint frame's origin
        KDL::Rotation link1Joint1FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
        grs::PoseCoordinates<KDL::Frame> link1Joint1FramePoseCoord(KDL::Frame(link1Joint1FrameOrientationCoord,link1Joint1FramePositionCoord));
        grs::PoseCoordinatesSemantics link1Joint1FramePoseSemantics(grs::Point("j1.Segment1.Link1"),grs::OrientationFrame("J1.Segment1.Link1"),grs::Body("Segment1.Link1"),
                                                                    grs::Point("f1.Segment1.Link1"),grs::OrientationFrame("F1.Segment1.Link1"),grs::Body("Segment1.Link1"),grs::OrientationFrame("F1.Segment1.Link1"));
        grs::Pose<KDL::Frame> link1Joint1FramePose(link1Joint1FramePoseSemantics, link1Joint1FramePoseCoord);
        
        //Joint2
        KDL::Vector link1Joint2FramePositionCoord = KDL::Vector(0.5,0,0); //position of joint frame's origin
        KDL::Rotation link1Joint2FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
        grs::PoseCoordinates<KDL::Frame> link1Joint2FramePoseCoord(KDL::Frame(link1Joint2FrameOrientationCoord,link1Joint2FramePositionCoord));
        grs::PoseCoordinatesSemantics link1Joint2FramePoseSemantics(grs::Point("j2.Segment1.Link1"),grs::OrientationFrame("J2.Segment1.Link1"),grs::Body("Segment1.Link1"),
                                                                    grs::Point("f1.Segment1.Link1"),grs::OrientationFrame("F1.Segment1.Link1"),grs::Body("Segment1.Link1"),grs::OrientationFrame("F1.Segment1.Link1"));
        grs::Pose<KDL::Frame> link1Joint2FramePose(link1Joint2FramePoseSemantics,link1Joint2FramePoseCoord);
        
        typedef std::vector< kdle::AttachmentFrame<grs::Pose<KDL::Frame> > > AttachmentFrames;
        AttachmentFrames frameList1;
        
        frameList1.push_back(kdle::createAttachmentFrame(link1Joint1FramePose, kdle::FrameType::JOINT));
        frameList1.push_back(kdle::createAttachmentFrame(link1Joint2FramePose, kdle::FrameType::JOINT));
        
        //LINK2 JOINT FRAMES
        //Joint1
        KDL::Vector link2Joint1FramePositionCoord = KDL::Vector(0.0,0,0); //position of joint frame's origin
        grs::Position<KDL::Vector> link2Joint1FramePosition(grs::Point("j1.Segment2.Link2"),grs::Body("Segment2.Link2"),
                                                            grs::Point("f2.Segment2.Link2"), grs::Body("Segment2.Link2"), grs::OrientationFrame("F2.Segment2.Link2"), link2Joint1FramePositionCoord);
        KDL::Rotation link2Joint1FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> link2Joint1FrameOrientation(grs::OrientationFrame("J1.Segment2.Link2"),grs::Body("Segment2.Link2"),
                                                                    grs::OrientationFrame("F2.Segment2.Link2"),grs::Body("Segment2.Link2"),grs::OrientationFrame("F2.Segment2.Link2"), link2Joint1FrameOrientationCoord);
        grs::Pose<KDL::Vector, KDL::Rotation> link2Joint1FramePose(link2Joint1FramePosition, link2Joint1FrameOrientation);
        
        //Joint2
        KDL::Vector link2Joint2FramePositionCoord = KDL::Vector(0.5,0,0); //position of joint frame's origin
        grs::Position<KDL::Vector> link2Joint2FramePosition(grs::Point("j2.Segment2.Link2"),grs::Body("Segment2.Link2"),
                                                            grs::Point("f2.Segment2.Link2"),grs::Body("Segment2.Link2"), grs::OrientationFrame("F2.Segment2.Link2"), link2Joint2FramePositionCoord);
        KDL::Rotation link2Joint2FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> link2Joint2FrameOrientation(grs::OrientationFrame("J2.Segment2.Link2"),grs::Body("Segment2.Link2"),
                                                                    grs::OrientationFrame("F2.Segment2.Link2"),grs::Body("Segment2.Link2"),grs::OrientationFrame("F2.Segment2.Link2"), link2Joint2FrameOrientationCoord);
        grs::Pose<KDL::Vector, KDL::Rotation> link2Joint2FramePose(link2Joint2FramePosition, link2Joint2FrameOrientation);
        
        typedef std::vector< kdle::AttachmentFrame<grs::Pose<KDL::Vector, KDL::Rotation> > > AttachmentFrames1;
        AttachmentFrames1 frameList2;
        
        
        frameList2.push_back(kdle::createAttachmentFrame(link2Joint1FramePose, kdle::FrameType::JOINT));
        frameList2.push_back(kdle::createAttachmentFrame(link2Joint2FramePose, kdle::FrameType::JOINT));
        
        //LINK3 JOINT FRAMES
        KDL::Vector link3Joint1FramePositionCoord = KDL::Vector(0.0,0,0); //position of joint frame's origin
        grs::Position<KDL::Vector> link3Joint1FramePosition(grs::Point("j1.Segment3.Link3"),grs::Body("Segment3.Link3"),
                                                            grs::Point("f3.Segment3.Link3"),grs::Body("Segment3.Link3"),grs::OrientationFrame("F3.Segment3.Link3"), link3Joint1FramePositionCoord);
        KDL::Rotation link3Joint1FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> link3Joint1FrameOrientation(grs::OrientationFrame("J1.Segment3.Link3"),grs::Body("Segment3.Link3"),
                                                                    grs::OrientationFrame("F3.Segment3.Link3"),grs::Body("Segment3.Link3"),grs::OrientationFrame("F3.Segment3.Link3"), link3Joint1FrameOrientationCoord);
        grs::Pose<KDL::Vector, KDL::Rotation> link3Joint1FramePose(link3Joint1FramePosition, link3Joint1FrameOrientation);
        
        //Joint1
//        KDL::Vector link3Joint1FramePositionCoord = KDL::Vector(0.0,0,0); //position of joint frame's origin
//        grs::Position<KDL::Vector> link3Joint1FramePosition("j1","Segment3.Link3","f3", "Segment3.Link3", "F3", link3Joint1FramePositionCoord);
//        KDL::Rotation link3Joint1FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
//        grs::Orientation<KDL::Rotation> link3Joint1FrameOrientation("J1","Segment3.Link3", "F3","Segment3.Link3","F3", link3Joint1FrameOrientationCoord);
//        grs::Pose<KDL::Vector, KDL::Rotation> link3Joint1FramePose(link3Joint1FramePosition, link3Joint1FrameOrientation);
        
        //Joint2
        KDL::Vector link3Joint2FramePositionCoord = KDL::Vector(0.5,0,0); //position of joint frame's origin
        grs::Position<KDL::Vector> link3Joint2FramePosition(grs::Point("j2.Segment3.Link3"),grs::Body("Segment3.Link3"),
                                                            grs::Point("f3.Segment3.Link3"),grs::Body("Segment3.Link3"),grs::OrientationFrame("F3.Segment3.Link3"), link3Joint2FramePositionCoord);
        KDL::Rotation link3Joint2FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> link3Joint2FrameOrientation(grs::OrientationFrame("J2.Segment3.Link3"),grs::Body("Segment3.Link3"),
                                                                    grs::OrientationFrame("F3.Segment3.Link3"),grs::Body("Segment3.Link3"),grs::OrientationFrame("F3.Segment3.Link3"), link3Joint2FrameOrientationCoord);
        grs::Pose<KDL::Vector, KDL::Rotation> link3Joint2FramePose(link3Joint2FramePosition, link3Joint2FrameOrientation);
        
        AttachmentFrames1 frameList3;
        frameList3.push_back(kdle::createAttachmentFrame(link3Joint1FramePose, kdle::FrameType::JOINT));
        frameList3.push_back(kdle::createAttachmentFrame(link3Joint2FramePose, kdle::FrameType::JOINT));
        
        //LINK4 JOINT FRAMES
        //Joint1
        KDL::Vector link4Joint1FramePositionCoord = KDL::Vector(0.0,0,0); //position of joint frame's origin
        grs::Position<KDL::Vector> link4Joint1FramePosition(grs::Point("j1.Segment4.Link4"),grs::Body("Segment4.Link4"),
                                                            grs::Point("f4.Segment4.Link4"),grs::Body("Segment4.Link4"),grs::OrientationFrame("F4.Segment4.Link4"), link4Joint1FramePositionCoord);
        KDL::Rotation link4Joint1FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> link4Joint1FrameOrientation(grs::OrientationFrame("J1.Segment4.Link4"),grs::Body("Segment4.Link4"),
                                                                    grs::OrientationFrame("F4.Segment4.Link4"),grs::Body("Segment4.Link4"),grs::OrientationFrame("F4.Segment4.Link4"), link4Joint1FrameOrientationCoord);
        grs::Pose<KDL::Vector, KDL::Rotation> link4Joint1FramePose(link4Joint1FramePosition, link4Joint1FrameOrientation);
        
        //Joint2
        KDL::Vector link4Joint2FramePositionCoord = KDL::Vector(0.5,0,0); //position of joint frame's origin
        grs::Position<KDL::Vector> link4Joint2FramePosition(grs::Point("j2.Segment4.Link4"),grs::Body("Segment4.Link4"),
                                                            grs::Point("f4.Segment4.Link4"),grs::Body("Segment4.Link4"),grs::OrientationFrame("F4.Segment4.Link4"), link4Joint2FramePositionCoord);
        KDL::Rotation link4Joint2FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> link4Joint2FrameOrientation(grs::OrientationFrame("J2.Segment4.Link4"),grs::Body("Segment4.Link4"),
                                                                    grs::OrientationFrame("F4.Segment4.Link4"),grs::Body("Segment4.Link4"),grs::OrientationFrame("F4.Segment4.Link4"), link4Joint2FrameOrientationCoord);
        grs::Pose<KDL::Vector, KDL::Rotation> link4Joint2FramePose(link4Joint2FramePosition, link4Joint2FrameOrientation);
        
        AttachmentFrames1 frameList4;
        frameList4.push_back(kdle::createAttachmentFrame(link4Joint1FramePose, kdle::FrameType::JOINT));
        frameList4.push_back(kdle::createAttachmentFrame(link4Joint2FramePose, kdle::FrameType::JOINT));
        frameList4.push_back(kdle::createAttachmentFrame(link4Joint2FramePose, kdle::FrameType::INERTIA));
        
        //Segment specification with one argument Pose
        //attaching frames at construction time
        kdle::Segment< grs::Pose<KDL::Frame> > segment0_grs("Segment1", link1, frameList1);
                
        //Segment specification with two argument Pose
        kdle::Segment< grs::Pose<KDL::Vector, KDL::Rotation> > segment2_grs("Segment2", link2, frameList2);
        kdle::Segment< grs::Pose<KDL::Vector, KDL::Rotation> > segment3_grs("Segment3", link3, frameList3);
        kdle::Segment< grs::Pose<KDL::Vector, KDL::Rotation> > segment4_grs("Segment4", link4, frameList4);

        kdle::JointProperties joint1_props, joint2_props;
        joint1_props = std::make_tuple(KDL::Joint::JointType::RotZ, 0.02, 150.0, -140.0);
        joint2_props = std::make_tuple(KDL::Joint::JointType::RotZ, 0.025, 145.0, -135.0);
        
        kdle::Joint< grs::Pose<KDL::Vector, KDL::Rotation> > joint2("Joint1", segment3_grs.getAttachmentFrames()[0], segment3_grs, segment2_grs.getAttachmentFrames()[1], segment2_grs, joint1_props);
        kdle::Joint< grs::Pose<KDL::Vector, KDL::Rotation> > joint3("Joint2", segment4_grs.getAttachmentFrames()[0], segment4_grs, segment3_grs.getAttachmentFrames()[1], segment3_grs,  joint2_props);
        
        
        kdle::TransmissionProperties trans_props = std::make_tuple(0.2,0.2,0.2);
        kdle::make_transmission(joint2,trans_props);
        grs::Pose<KDL::Vector, KDL::Rotation> currentJointPose;
        grs::Twist<KDL::Vector, KDL::Vector> currentJointTwist;
        grs::Twist<KDL::Twist> twisttemp;
        //joint value is of type vector whose size changes according to the joint's DoF
        std::vector<double> jointvalue(1,M_PI/4.0);
        joint2.getPoseOfJointFrames(jointvalue, currentJointPose );
        joint3.getPoseOfJointFrames(jointvalue, currentJointPose );
//        joint2.getCurrentTipToPredecessorTipPose(jointvalue, currentJointPose );
//        joint2.getCurrentRootToPredecessorTipTwist(jointvalue, currentJointTwist);
//        joint2.getCurrentTipToPredecessorTipTwist(jointvalue, currentJointTwist);
//        joint2.getCurrentRootToPredecessorTipTwist(jointvalue, twisttemp);
        
        std::vector< kdle::Joint< grs::Pose<KDL::Vector, KDL::Rotation> > > jointlist;
        jointlist.push_back(joint2);
        jointlist.push_back(joint3);
        
        kdle::KinematicChain< grs::Pose<KDL::Vector, KDL::Rotation> > mychain("MyKinematicChain", jointlist);
        int tempNr = mychain.getNrOfJoints();
        mychain.addJoint(joint3);
        tempNr = mychain.getNrOfSegments();
       
        
    //~SEGMENT METADATA
    //Computational operation
        kdle::transform<kdle::tree_iterator, kdle::pose> comp1;
    //Traversal policy
        kdle::DFSPolicy_ver2< kdle::KinematicChain< grs::Pose<KDL::Vector, KDL::Rotation> > > policy;
    //Traversal operation
        kdle::traverseGraph_ver2(mychain, comp1,policy);
    return 0;
}

