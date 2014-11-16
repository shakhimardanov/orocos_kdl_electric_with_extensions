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
        KDL::Vector rootFramePosition1 = KDL::Vector(0.1,0,0); //position of joint frame's origin
        KDL::Rotation rootFrameOrientation1 = KDL::Rotation::RotZ(0.0);
        grs::PoseCoordinates<KDL::Frame> rootFramePoseCoord1(KDL::Frame(rootFrameOrientation1, rootFramePosition1));
        grs::PoseCoordinatesSemantics rootFramePoseSemantics1("f1","F1","Segment1.Link1","b","B","Base","B");
        grs::Pose<KDL::Frame> rootFramePose1(rootFramePoseSemantics1, rootFramePoseCoord1);
        
        //tip frame wrt base
        KDL::Vector tipFramePosition1 = KDL::Vector(-0.4, 0.0, 0.0);
        KDL::Rotation tipFrameOrientation1 = KDL::Rotation::Identity();
        grs::PoseCoordinates<KDL::Frame> tipFramePoseCoord1(KDL::Frame(tipFrameOrientation1, tipFramePosition1));
        grs::PoseCoordinatesSemantics tipFramePoseSemantics1("l1","L1","Segment1.Link1","b","B","Base","B");
        grs::Pose<KDL::Frame> tipFramePose1(tipFramePoseSemantics1, tipFramePoseCoord1);
        
        //Link specification with one argument Pose
        kdle::Link< grs::Pose<KDL::Frame> > link1("link1", rootFramePose1, tipFramePose1);
        
        //two argument pose
        //LINK2 FRAMES
        // root frame pose wrt base
        KDL::Vector rootFramePositionCoord2 = KDL::Vector(-0.1,0,0); //position of joint frame's origin
        grs::Position<KDL::Vector> rootFramePosition2("f2","Segment2.Link2","b", "Base", "B", rootFramePositionCoord2);
        KDL::Rotation rootFrameOrientationCoord2 = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> rootFrameOrientation2("F2","Segment2.Link2", "B","Base","B", rootFrameOrientationCoord2);
        grs::Pose<KDL::Vector, KDL::Rotation> rootFramePose2(rootFramePosition2, rootFrameOrientation2);
        
        //tip frame wrt base
        KDL::Vector tipFramePositionCoord2 = KDL::Vector(0.4, 0.0, 0.0);
        grs::Position<KDL::Vector> tipFramePosition2("l2","Segment2.Link2","b", "Base", "B", tipFramePositionCoord2);
        KDL::Rotation tipFrameOrientationCoord2 = KDL::Rotation::Identity();
        grs::Orientation<KDL::Rotation> tipFrameOrientation2("L2","Segment2.Link2", "B","Base","B", tipFrameOrientationCoord2);
        grs::Pose<KDL::Vector, KDL::Rotation> tipFramePose2(tipFramePosition2, tipFrameOrientation2);
        
        //Link specification with two argument Pose
        kdle::Link< grs::Pose<KDL::Vector, KDL::Rotation> > link2("link2", rootFramePose2, tipFramePose2);
        
        //LINK3 FRAMES
        // root frame pose wrt base
        KDL::Vector rootFramePositionCoord3 = KDL::Vector(0,0,0); //position of joint frame's origin
        grs::Position<KDL::Vector> rootFramePosition3("f3","Segment3.Link3","b", "Base", "B", rootFramePositionCoord3);
        KDL::Rotation rootFrameOrientationCoord3 = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> rootFrameOrientation3("F3","Segment3.Link3", "B","Base","B", rootFrameOrientationCoord3);
        grs::Pose<KDL::Vector, KDL::Rotation> rootFramePose3(rootFramePosition3, rootFrameOrientation3);
        
        //tip frame wrt base
        KDL::Vector tipFramePositionCoord3 = KDL::Vector(-0.4, 0.0, 0.0);
        grs::Position<KDL::Vector> tipFramePosition3("l3","Segment3.Link3","b", "Base", "B", tipFramePositionCoord3);
        KDL::Rotation tipFrameOrientationCoord3 = KDL::Rotation::Identity();
        grs::Orientation<KDL::Rotation> tipFrameOrientation3("L3","Segment3.Link3", "B","Base","B", tipFrameOrientationCoord3);
        grs::Pose<KDL::Vector, KDL::Rotation> tipFramePose3(tipFramePosition3, tipFrameOrientation3);
        
        //Link specification with two argument Pose
        kdle::Link< grs::Pose<KDL::Vector, KDL::Rotation> > link3("link3", rootFramePose3, tipFramePose3);
        
        //LINK4 FRAMES
        // root frame pose wrt base
        KDL::Vector rootFramePositionCoord4 = KDL::Vector(0,0,0); //position of joint frame's origin
        grs::Position<KDL::Vector> rootFramePosition4("f4","Segment4.Link4","b", "Base", "B", rootFramePositionCoord4);
        KDL::Rotation rootFrameOrientationCoord4 = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> rootFrameOrientation4("F4","Segment3.Link4", "B","Base","B", rootFrameOrientationCoord4);
        grs::Pose<KDL::Vector, KDL::Rotation> rootFramePose4(rootFramePosition4, rootFrameOrientation4);
        
        //tip frame wrt base
        KDL::Vector tipFramePositionCoord4 = KDL::Vector(0.4, 0.0, 0.0);
        grs::Position<KDL::Vector> tipFramePosition4("l4","Segment4.Link4","b", "Base", "B", tipFramePositionCoord4);
        KDL::Rotation tipFrameOrientationCoord4 = KDL::Rotation::Identity();
        grs::Orientation<KDL::Rotation> tipFrameOrientation4("L4","Segment4.Link4", "B","Base","B", tipFrameOrientationCoord4);
        grs::Pose<KDL::Vector, KDL::Rotation> tipFramePose4(tipFramePosition4, tipFrameOrientation4);
        
        //Link specification with two argument Pose
        kdle::Link< grs::Pose<KDL::Vector, KDL::Rotation> > link4("link4", rootFramePose4, tipFramePose4);
        
        //ATTACHMENT FRAMES OF A SEGMENT
        
        //LINK1 JOINT FRAMES
        //Joint1
        KDL::Vector link1Joint1FramePositionCoord = KDL::Vector(0.1,0,0); //position of joint frame's origin
        KDL::Rotation link1Joint1FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
        grs::PoseCoordinates<KDL::Frame> link1Joint1FramePoseCoord(KDL::Frame(link1Joint1FrameOrientationCoord,link1Joint1FramePositionCoord));
        grs::PoseCoordinatesSemantics link1Joint1FramePoseSemantics("j1","J1","Segment1.Link1","b","B","Base","B");
        grs::Pose<KDL::Frame> link1Joint1FramePose(link1Joint1FramePoseSemantics, link1Joint1FramePoseCoord);
        
        //Joint2
        KDL::Vector link1Joint2FramePositionCoord = KDL::Vector(-0.4,0,0); //position of joint frame's origin
        KDL::Rotation link1Joint2FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
        grs::PoseCoordinates<KDL::Frame> link1Joint2FramePoseCoord(KDL::Frame(link1Joint2FrameOrientationCoord,link1Joint2FramePositionCoord));
        grs::PoseCoordinatesSemantics link1Joint2FramePoseSemantics("j2","J2","Segment1.Link1","b","B","Base","B");
        grs::Pose<KDL::Frame> link1Joint2FramePose(link1Joint2FramePoseSemantics,link1Joint2FramePoseCoord);
        
        typedef std::vector< kdle::AttachmentFrame<grs::Pose<KDL::Frame> > > AttachmentFrames;
        AttachmentFrames frameList1;
        
        frameList1.push_back(kdle::createAttachmentFrame(link1Joint1FramePose, kdle::FrameType::JOINT));
        frameList1.push_back(kdle::createAttachmentFrame(link1Joint2FramePose, kdle::FrameType::JOINT));
        
        //LINK2 JOINT FRAMES
        //Joint1
        KDL::Vector link2Joint1FramePositionCoord = KDL::Vector(0.0,0,0); //position of joint frame's origin
        grs::Position<KDL::Vector> link2Joint1FramePosition("j1","Segment2.Link2","b", "Base", "B", link2Joint1FramePositionCoord);
        KDL::Rotation link2Joint1FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> link2Joint1FrameOrientation("J1","Segment2.Link2", "B","Base","B", link2Joint1FrameOrientationCoord);
        grs::Pose<KDL::Vector, KDL::Rotation> link2Joint1FramePose(link2Joint1FramePosition, link2Joint1FrameOrientation);
        
        //Joint2
        KDL::Vector link2Joint2FramePositionCoord = KDL::Vector(0.4,0,0); //position of joint frame's origin
        grs::Position<KDL::Vector> link2Joint2FramePosition("j2","Segment2.Link2","b", "Base", "B", link2Joint2FramePositionCoord);
        KDL::Rotation link2Joint2FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> link2Joint2FrameOrientation("J2","Segment2.Link2", "B","Base","B", link2Joint2FrameOrientationCoord);
        grs::Pose<KDL::Vector, KDL::Rotation> link2Joint2FramePose(link2Joint2FramePosition, link2Joint2FrameOrientation);
        
        typedef std::vector< kdle::AttachmentFrame<grs::Pose<KDL::Vector, KDL::Rotation> > > AttachmentFrames1;
        AttachmentFrames1 frameList2;
        
        frameList2.push_back(kdle::createAttachmentFrame(link2Joint1FramePose, kdle::FrameType::JOINT));
        frameList2.push_back(kdle::createAttachmentFrame(link2Joint2FramePose, kdle::FrameType::JOINT));
        
        //LINK3 JOINT FRAMES
        //Joint1
        KDL::Vector link3Joint1FramePositionCoord = KDL::Vector(0.0,0,0); //position of joint frame's origin
        grs::Position<KDL::Vector> link3Joint1FramePosition("j1","Segment3.Link3","b", "Base", "B", link3Joint1FramePositionCoord);
        KDL::Rotation link3Joint1FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> link3Joint1FrameOrientation("J1","Segment3.Link3", "B","Base","B", link3Joint1FrameOrientationCoord);
        grs::Pose<KDL::Vector, KDL::Rotation> link3Joint1FramePose(link3Joint1FramePosition, link3Joint1FrameOrientation);
        
        //Joint2
        KDL::Vector link3Joint2FramePositionCoord = KDL::Vector(-0.2,0,0); //position of joint frame's origin
        grs::Position<KDL::Vector> link3Joint2FramePosition("j2","Segment3.Link3","b", "Base", "B", link3Joint2FramePositionCoord);
        KDL::Rotation link3Joint2FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> link3Joint2FrameOrientation("J2","Segment3.Link3", "B","Base","B", link3Joint2FrameOrientationCoord);
        grs::Pose<KDL::Vector, KDL::Rotation> link3Joint2FramePose(link3Joint2FramePosition, link3Joint2FrameOrientation);
        
        AttachmentFrames1 frameList3;
        frameList3.push_back(kdle::createAttachmentFrame(link3Joint1FramePose, kdle::FrameType::JOINT));
        frameList3.push_back(kdle::createAttachmentFrame(link3Joint2FramePose, kdle::FrameType::JOINT));
        
        //LINK4 JOINT FRAMES
        //Joint1
        KDL::Vector link4Joint1FramePositionCoord = KDL::Vector(0.0,0,0); //position of joint frame's origin
        grs::Position<KDL::Vector> link4Joint1FramePosition("j1","Segment4.Link4","b", "Base", "B", link4Joint1FramePositionCoord);
        KDL::Rotation link4Joint1FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> link4Joint1FrameOrientation("J1","Segment4.Link4", "B","Base","B", link4Joint1FrameOrientationCoord);
        grs::Pose<KDL::Vector, KDL::Rotation> link4Joint1FramePose(link4Joint1FramePosition, link4Joint1FrameOrientation);
        
        //Joint2
        KDL::Vector link4Joint2FramePositionCoord = KDL::Vector(0.4,0,0); //position of joint frame's origin
        grs::Position<KDL::Vector> link4Joint2FramePosition("j2","Segment4.Link4","b", "Base", "B", link4Joint2FramePositionCoord);
        KDL::Rotation link4Joint2FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> link4Joint2FrameOrientation("J2","Segment4.Link4", "B","Base","B", link4Joint2FrameOrientationCoord);
        grs::Pose<KDL::Vector, KDL::Rotation> link4Joint2FramePose(link4Joint2FramePosition, link4Joint2FrameOrientation);
        
        AttachmentFrames1 frameList4;
        frameList4.push_back(kdle::createAttachmentFrame(link4Joint1FramePose, kdle::FrameType::JOINT));
        frameList4.push_back(kdle::createAttachmentFrame(link4Joint2FramePose, kdle::FrameType::JOINT));
        frameList4.push_back(kdle::createAttachmentFrame(link4Joint2FramePose, kdle::FrameType::INERTIA));
        
        //Segment specification with one argument Pose
        //attaching frames at construction time
        kdle::Segment< grs::Pose<KDL::Frame> > segment0_grs("segment1", link1, frameList1);
                
        //Segment specification with two argument Pose
        kdle::Segment< grs::Pose<KDL::Vector, KDL::Rotation> > segment2_grs("segment2", link2, frameList2);
        kdle::Segment< grs::Pose<KDL::Vector, KDL::Rotation> > segment3_grs("segment3", link3, frameList3);
        kdle::Segment< grs::Pose<KDL::Vector, KDL::Rotation> > segment4_grs("segment4", link4, frameList4);

        kdle::JointProperties joint1_props, joint2_props;
        joint1_props = std::make_tuple(KDL::Joint::JointType::RotZ, 0.02, 150.0, -140.0);
        joint2_props = std::make_tuple(KDL::Joint::JointType::RotZ, 0.025, 145.0, -135.0);
        
        kdle::Joint< grs::Pose<KDL::Vector, KDL::Rotation> > joint2("joint1", frameList2[1], frameList3[0], joint1_props);
        kdle::Joint< grs::Pose<KDL::Vector, KDL::Rotation> > joint3("joint2", segment3_grs.getAttachmentFrames()[1], segment4_grs.getAttachmentFrames()[0], joint2_props);
        
        
        kdle::TransmissionProperties trans_props = std::make_tuple(0.2,0.2,0.2);
        kdle::make_transmission(joint2,trans_props);
        grs::Pose<KDL::Vector, KDL::Rotation> currentJointPose;
        std::vector<double> jointvalue(1,0.2);
        joint2.getPose(jointvalue, currentJointPose );
        
        std::vector< kdle::Joint< grs::Pose<KDL::Vector, KDL::Rotation> > > jointlist;
        jointlist.push_back(joint2);
        jointlist.push_back(joint3);
        kdle::Chain< grs::Pose<KDL::Vector, KDL::Rotation> > mychain("MyChain", jointlist);
        
    //~SEGMENT METADATA
    //Computational operation
        kdle::transform<kdle::tree_iterator, kdle::pose> comp1;
    //Traversal policy
        kdle::DFSPolicy_ver2< kdle::Chain< grs::Pose<KDL::Vector, KDL::Rotation> > > policy;
    //Traversal operation
        kdle::traverseGraph_ver2(mychain, comp1,policy);
    return 0;
}

