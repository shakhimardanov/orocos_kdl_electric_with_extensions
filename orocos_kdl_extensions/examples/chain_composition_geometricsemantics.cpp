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

/* This example show how geometric relations semantics library can be used with 
 kdl extensions library. Here a kinematic chain instance uses geometric 
 semantics library primitives. The constructed chain is then used kinematic or
 dynamics computations which are defined using kdl extensions library*/

#include <kdl_extensions/chain_geometric_primitives.hpp>
#include <kdl_extensions/functionalcomputation_kdl.hpp>

using namespace std;

typedef struct
{
    std::string point;
    std::string frame;
    std::string body;
} SemanticData;

typedef struct
{
    KDL::Vector link_root_position_coord;
    KDL::Rotation link_root_orientation_coord;
    KDL::Vector link_tip_position_coord;
    KDL::Rotation link_tip_orientation_coord;
} SegmentGeometricData;


typedef struct
{
    KDL::Vector joint_origin_position_coord;
    KDL::Rotation joint_origin_orientation_coord;
} SegmentJointFrameGeometricData;

void initSegmentFrameSemantics(SemanticData *bodyframes, const unsigned int size)
{
    bodyframes[0].point = "b.Base";
    bodyframes[0].frame = "B.Base";
    bodyframes[0].body = "Base";
    
    bodyframes[1].point = "f1.Segment1.Link1";
    bodyframes[1].frame = "F1.Segment1.Link1";
    bodyframes[1].body = "Segment1.Link1";
   
    bodyframes[2].point = "l1.Segment1.Link1";
    bodyframes[2].frame = "L1.Segment1.Link1";
    bodyframes[2].body = "Segment1.Link1";
   
    bodyframes[3].point = "f2.Segment2.Link2";
    bodyframes[3].frame = "F2.Segment2.Link2";
    bodyframes[3].body = "Segment2.Link2";

    bodyframes[4].point = "l2.Segment2.Link2";
    bodyframes[4].frame = "L2.Segment2.Link2";
    bodyframes[4].body = "Segment2.Link2";

    bodyframes[5].point = "f3.Segment3.Link3";
    bodyframes[5].frame = "F3.Segment3.Link3";
    bodyframes[5].body = "Segment3.Link3";
    
    bodyframes[6].point = "l3.Segment3.Link3";
    bodyframes[6].frame = "L3.Segment3.Link3";
    bodyframes[6].body = "Segment3.Link3";

    bodyframes[7].point = "f4.Segment4.Link4";
    bodyframes[7].frame = "F4.Segment4.Link4";
    bodyframes[7].body = "Segment4.Link4";

    bodyframes[8].point = "l4.Segment4.Link4";
    bodyframes[8].frame = "L4.Segment4.Link4";
    bodyframes[8].body = "Segment4.Link4";
    
    bodyframes[9].point = "f5.Segment5.Link5";
    bodyframes[9].frame = "F5.Segment5.Link5";
    bodyframes[9].body = "Segment5.Link5";    

    bodyframes[10].point = "l5.Segment5.Link5";
    bodyframes[10].frame = "L5.Segment5.Link5";
    bodyframes[10].body = "Segment5.Link5";    
}

void initSegmentJointFrameSemanics(SemanticData *bodyframes, const unsigned int size)
{
    bodyframes[0].point = "j1.Segment1.Link1";
    bodyframes[0].frame = "J1.Segment1.Link1";
    bodyframes[0].body = "Segment1.Link1";
    
    bodyframes[1].point = "j2.Segment1.Link1";
    bodyframes[1].frame = "J2.Segment1.Link1";
    bodyframes[1].body = "Segment1.Link1";
    
    bodyframes[2].point = "j3.Segment1.Link1";
    bodyframes[2].frame = "J3.Segment1.Link1";
    bodyframes[2].body = "Segment1.Link1";
    
    bodyframes[3].point = "j1.Segment2.Link2";
    bodyframes[3].frame = "J1.Segment2.Link2";
    bodyframes[3].body = "Segment2.Link2";
    
    bodyframes[4].point = "j2.Segment2.Link2";
    bodyframes[4].frame = "J2.Segment2.Link2";
    bodyframes[4].body = "Segment2.Link2";
    
    bodyframes[5].point = "j3.Segment2.Link2";
    bodyframes[5].frame = "J3.Segment2.Link2";
    bodyframes[5].body = "Segment2.Link2";

    bodyframes[6].point = "j1.Segment3.Link3";
    bodyframes[6].frame = "J1.Segment3.Link3";
    bodyframes[6].body = "Segment3.Link3";
    
    bodyframes[7].point = "j2.Segment3.Link3";
    bodyframes[7].frame = "J2.Segment3.Link3";
    bodyframes[7].body = "Segment3.Link3";
    
    bodyframes[8].point = "j3.Segment3.Link3";
    bodyframes[8].frame = "J3.Segment3.Link3";
    bodyframes[8].body = "Segment3.Link3";

    bodyframes[9].point = "j1.Segment4.Link4";
    bodyframes[9].frame = "J1.Segment4.Link4";
    bodyframes[9].body = "Segment4.Link4";
    
    bodyframes[10].point = "j2.Segment4.Link4";
    bodyframes[10].frame = "J2.Segment4.Link4";
    bodyframes[10].body = "Segment4.Link4";
    
    bodyframes[11].point = "j3.Segment4.Link4";
    bodyframes[11].frame = "J3.Segment4.Link4";
    bodyframes[11].body = "Segment4.Link4";

    bodyframes[12].point = "j1.Segment5.Link5";
    bodyframes[12].frame = "J1.Segment5.Link5";
    bodyframes[12].body = "Segment5.Link5";
    
    bodyframes[13].point = "j2.Segment5.Link5";
    bodyframes[13].frame = "J2.Segment5.Link5";
    bodyframes[13].body = "Segment5.Link5";
    
    bodyframes[14].point = "j3.Segment5.Link5";
    bodyframes[14].frame = "J3.Segment5.Link5";
    bodyframes[14].body = "Segment5.Link5";
};

void initSegmentFrameGeometry(SegmentGeometricData *segmentgeoms, const unsigned int size)
{
    segmentgeoms[0].link_root_position_coord = KDL::Vector(0,0,0);
    segmentgeoms[0].link_root_orientation_coord =  KDL::Rotation::RotZ(0.0);
    segmentgeoms[0].link_tip_position_coord = KDL::Vector(0.5,0.0,0.0);
    segmentgeoms[0].link_tip_orientation_coord = KDL::Rotation::Identity();
    
    segmentgeoms[1].link_root_position_coord = KDL::Vector(0.1,0,0);
    segmentgeoms[1].link_root_orientation_coord =  KDL::Rotation::RotZ(0.0);
    segmentgeoms[1].link_tip_position_coord = KDL::Vector(0.5,0.0,0.0);
    segmentgeoms[1].link_tip_orientation_coord = KDL::Rotation::Identity();

    segmentgeoms[2].link_root_position_coord = KDL::Vector(0.2,0,0);
    segmentgeoms[2].link_root_orientation_coord =  KDL::Rotation::RotZ(0.0);
    segmentgeoms[2].link_tip_position_coord = KDL::Vector(0.5,0.0,0.0);
    segmentgeoms[2].link_tip_orientation_coord = KDL::Rotation::Identity();

    segmentgeoms[3].link_root_position_coord = KDL::Vector(0,0,0.15);
    segmentgeoms[3].link_root_orientation_coord =  KDL::Rotation::RotZ(0.0);
    segmentgeoms[3].link_tip_position_coord = KDL::Vector(0.0,0.0,0.65);
    segmentgeoms[3].link_tip_orientation_coord = KDL::Rotation::Identity();

    segmentgeoms[4].link_root_position_coord = KDL::Vector(0,0,0);
    segmentgeoms[4].link_root_orientation_coord =  KDL::Rotation::RotZ(0.0);
    segmentgeoms[4].link_tip_position_coord = KDL::Vector(0.0,0.0,-0.75);
    segmentgeoms[4].link_tip_orientation_coord = KDL::Rotation::Identity();
}

void initSegmentJointFrameGeometry(SegmentJointFrameGeometricData *segmentgeoms, const unsigned int size)
{
    segmentgeoms[0].joint_origin_position_coord = KDL::Vector(0,0,0);
    segmentgeoms[0].joint_origin_orientation_coord =  KDL::Rotation::RotZ(0.0);
    
    segmentgeoms[1].joint_origin_position_coord = KDL::Vector(0.35,0,0);
    segmentgeoms[1].joint_origin_orientation_coord =  KDL::Rotation::RotZ(0.0);
    
    segmentgeoms[2].joint_origin_position_coord = KDL::Vector(0,0,0);
    segmentgeoms[2].joint_origin_orientation_coord =  KDL::Rotation::RotZ(0.0);
    
    segmentgeoms[3].joint_origin_position_coord = KDL::Vector(0.15,0,0);
    segmentgeoms[3].joint_origin_orientation_coord =  KDL::Rotation::RotZ(0.0);
    
    segmentgeoms[4].joint_origin_position_coord = KDL::Vector(0.45,0,0);
    segmentgeoms[4].joint_origin_orientation_coord =  KDL::Rotation::RotZ(0.0);
    
    segmentgeoms[5].joint_origin_position_coord = KDL::Vector(0,0,0);
    segmentgeoms[5].joint_origin_orientation_coord =  KDL::Rotation::RotZ(0.0);
    
    segmentgeoms[6].joint_origin_position_coord = KDL::Vector(0.1,0,0);
    segmentgeoms[6].joint_origin_orientation_coord =  KDL::Rotation::RotZ(0.0);
    
    segmentgeoms[7].joint_origin_position_coord = KDL::Vector(0.35,0,0);
    segmentgeoms[7].joint_origin_orientation_coord =  KDL::Rotation::RotZ(0.0);
    
    segmentgeoms[8].joint_origin_position_coord = KDL::Vector(0,0,0);
    segmentgeoms[8].joint_origin_orientation_coord =  KDL::Rotation::RotZ(0.0);
    
    segmentgeoms[9].joint_origin_position_coord = KDL::Vector(0,0,0);
    segmentgeoms[9].joint_origin_orientation_coord =  KDL::Rotation::RotZ(0.0);

    segmentgeoms[10].joint_origin_position_coord = KDL::Vector(0,0,0.15);
    segmentgeoms[10].joint_origin_orientation_coord =  KDL::Rotation::RotZ(0.0);
    
    segmentgeoms[11].joint_origin_position_coord = KDL::Vector(0,0,0.25);
    segmentgeoms[11].joint_origin_orientation_coord =  KDL::Rotation::RotZ(0.0);
    
    segmentgeoms[12].joint_origin_position_coord = KDL::Vector(0,0,0);
    segmentgeoms[12].joint_origin_orientation_coord =  KDL::Rotation::RotZ(0.0);
    
    segmentgeoms[13].joint_origin_position_coord = KDL::Vector(0,0,-0.2);
    segmentgeoms[13].joint_origin_orientation_coord =  KDL::Rotation::RotZ(0.0);
    
    segmentgeoms[14].joint_origin_position_coord = KDL::Vector(0,0,-0.1);
    segmentgeoms[14].joint_origin_orientation_coord =  KDL::Rotation::RotZ(0.0);
}

int main(int argc, char** argv)
{
    unsigned int numberOfSegments = 5;
    unsigned int numberOfJointFramesPerSegment = 3;
    unsigned int numberOfSegmentFrames = 2*numberOfSegments +1;
    unsigned int numberOfJointFrames = numberOfJointFramesPerSegment*numberOfSegments;

    SemanticData segmentframesemantcs[numberOfSegmentFrames];
    initSegmentFrameSemantics(segmentframesemantcs,numberOfSegmentFrames);
    SemanticData segmentjointframesemantics[numberOfJointFrames];
    initSegmentJointFrameSemanics(segmentjointframesemantics,numberOfJointFrames);
    SegmentGeometricData segmentframecoord[numberOfSegmentFrames];
    initSegmentFrameGeometry(segmentframecoord,numberOfSegmentFrames);
    SegmentJointFrameGeometricData segmentjointframecoord[numberOfJointFrames];
    initSegmentJointFrameGeometry(segmentjointframecoord,numberOfJointFrames);
    
    std::vector< grs::Pose<KDL::Vector, KDL::Rotation> > rootFramePoses;
    std::vector< grs::Pose<KDL::Vector, KDL::Rotation> > tipFramePoses;
    std::vector< grs::Pose<KDL::Vector, KDL::Rotation> > jointFramePoses;
    
    //SEGMENT METADATA
    //two argument pose
    //LINK1 FRAMES
        // root frame pose wrt base
        KDL::Vector rootFramePositionCoord1 = KDL::Vector(0.0,0,0); //position of joint frame's origin
        grs::Position<KDL::Vector> rootFramePosition1(grs::Point("f1.Segment1.Link1"),grs::Body("Segment1.Link1"),
                                                      grs::Point("b.Base"), grs::Body("Base"), grs::OrientationFrame("B.Base"), rootFramePositionCoord1);
        KDL::Rotation rootFrameOrientationCoord1 = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> rootFrameOrientation1(grs::OrientationFrame("F1.Segment1.Link1"),grs::Body("Segment1.Link1"), 
                                                              grs::OrientationFrame("B.Base"),grs::Body("Base"), grs::OrientationFrame("B.Base"), rootFrameOrientationCoord1);
        grs::Pose<KDL::Vector, KDL::Rotation> rootFramePose1(rootFramePosition1, rootFrameOrientation1);
        
        //tip frame wrt base
        KDL::Vector tipFramePositionCoord1 = KDL::Vector(0.0, 0.5, 0.0);
        grs::Position<KDL::Vector> tipFramePosition1(grs::Point("l1.Segment1.Link1"),grs::Body("Segment1.Link1"),
                                                     grs::Point("f1.Segment1.Link1"),grs::Body("Segment1.Link1"), grs::OrientationFrame("F1.Segment1.Link1"), tipFramePositionCoord1);
        KDL::Rotation tipFrameOrientationCoord1 = KDL::Rotation::Identity();
        grs::Orientation<KDL::Rotation> tipFrameOrientation1(grs::OrientationFrame("L1.Segment1.Link1"),grs::Body("Segment1.Link1"),
                                                             grs::OrientationFrame("F1.Segment1.Link1"),grs::Body("Segment1.Link1"), grs::OrientationFrame("F1.Segment1.Link1"), tipFrameOrientationCoord1);
        grs::Pose<KDL::Vector, KDL::Rotation> tipFramePose1(tipFramePosition1, tipFrameOrientation1);
        
        //Link specification with one argument Pose
        kdle::Link< grs::Pose<KDL::Vector, KDL::Rotation> > link1("Link1", rootFramePose1, tipFramePose1);
        
    //LINK2 FRAMES
        // root frame pose wrt base
        KDL::Vector rootFramePositionCoord2 = KDL::Vector(0.1,0,0); //position of joint frame's origin
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
        KDL::Vector rootFramePositionCoord3 = KDL::Vector(0.2,0,0); //position of joint frame's origin
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
        KDL::Vector rootFramePositionCoord4 = KDL::Vector(0.0,0,0.15); //position of joint frame's origin
        grs::Position<KDL::Vector> rootFramePosition4(grs::Point("f4.Segment4.Link4"),grs::Body("Segment4.Link4"),
                                                      grs::Point("b.Base"), grs::Body("Base"), grs::OrientationFrame("B.Base"), rootFramePositionCoord4);
        KDL::Rotation rootFrameOrientationCoord4 = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> rootFrameOrientation4(grs::OrientationFrame("F4.Segment4.Link4"),grs::Body("Segment4.Link4"),
                                                              grs::OrientationFrame("B.Base"),grs::Body("Base"), grs::OrientationFrame("B.Base"), rootFrameOrientationCoord4);
        grs::Pose<KDL::Vector, KDL::Rotation> rootFramePose4(rootFramePosition4, rootFrameOrientation4);
        
        //tip frame wrt base
        KDL::Vector tipFramePositionCoord4 = KDL::Vector(0.0, 0.0, 0.650);
        grs::Position<KDL::Vector> tipFramePosition4(grs::Point("l4.Segment4.Link4"),grs::Body("Segment4.Link4"),
                                                     grs::Point("f4.Segment4.Link4"),grs::Body("Segment4.Link4"), grs::OrientationFrame("F4.Segment4.Link4"), tipFramePositionCoord4);
        KDL::Rotation tipFrameOrientationCoord4 = KDL::Rotation::Identity();
        grs::Orientation<KDL::Rotation> tipFrameOrientation4(grs::OrientationFrame("L4.Segment4.Link4"),grs::Body("Segment4.Link4"),
                                                             grs::OrientationFrame("F4.Segment4.Link4"),grs::Body("Segment4.Link4"),grs::OrientationFrame("F4.Segment4.Link4"), tipFrameOrientationCoord4);
        grs::Pose<KDL::Vector, KDL::Rotation> tipFramePose4(tipFramePosition4, tipFrameOrientation4);
        
        //Link specification with two argument Pose
        kdle::Link< grs::Pose<KDL::Vector, KDL::Rotation> > link4("Link4", rootFramePose4, tipFramePose4);
        

    //LINK5 FRAMES
        // root frame pose wrt base
        KDL::Vector rootFramePositionCoord5 = KDL::Vector(0.0,0,0.0); //position of joint frame's origin
        grs::Position<KDL::Vector> rootFramePosition5(grs::Point("f5.Segment5.Link5"),grs::Body("Segment5.Link5"),
                                                      grs::Point("b.Base"), grs::Body("Base"), grs::OrientationFrame("B.Base"), rootFramePositionCoord5);
        KDL::Rotation rootFrameOrientationCoord5 = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> rootFrameOrientation5(grs::OrientationFrame("F5.Segment5.Link5"),grs::Body("Segment5.Link5"),
                                                              grs::OrientationFrame("B.Base"),grs::Body("Base"), grs::OrientationFrame("B.Base"), rootFrameOrientationCoord5);
        grs::Pose<KDL::Vector, KDL::Rotation> rootFramePose5(rootFramePosition5, rootFrameOrientation5);
        
        //tip frame wrt base
        KDL::Vector tipFramePositionCoord5 = KDL::Vector(0.0, 0.0, 0.750);
        grs::Position<KDL::Vector> tipFramePosition5(grs::Point("l5.Segment5.Link5"),grs::Body("Segment5.Link5"),
                                                     grs::Point("f5.Segment5.Link5"),grs::Body("Segment5.Link5"), grs::OrientationFrame("F5.Segment5.Link5"), tipFramePositionCoord5);
        KDL::Rotation tipFrameOrientationCoord5 = KDL::Rotation::Identity();
        grs::Orientation<KDL::Rotation> tipFrameOrientation5(grs::OrientationFrame("L5.Segment5.Link5"),grs::Body("Segment5.Link5"),
                                                             grs::OrientationFrame("F5.Segment5.Link5"),grs::Body("Segment5.Link5"),grs::OrientationFrame("F5.Segment5.Link5"), tipFrameOrientationCoord5);
        grs::Pose<KDL::Vector, KDL::Rotation> tipFramePose5(tipFramePosition5, tipFrameOrientation5);
        
        //Link specification with two argument Pose
        kdle::Link< grs::Pose<KDL::Vector, KDL::Rotation> > link5("Link5", rootFramePose5, tipFramePose5);


        //ATTACHMENT FRAMES OF A SEGMENT
        
    //LINK1 JOINT FRAMES
        //Joint1
        KDL::Vector link1Joint1FramePositionCoord = KDL::Vector(0.0,0,0); //position of joint frame's origin
        grs::Position<KDL::Vector> link1Joint1FramePosition(grs::Point("j1.Segment1.Link1"),grs::Body("Segment1.Link1"),
                                                            grs::Point("f1.Segment1.Link1"), grs::Body("Segment1.Link1"), grs::OrientationFrame("F1.Segment1.Link1"), link1Joint1FramePositionCoord);
        KDL::Rotation link1Joint1FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> link1Joint1FrameOrientation(grs::OrientationFrame("J1.Segment1.Link1"),grs::Body("Segment1.Link1"),
                                                                    grs::OrientationFrame("F1.Segment1.Link1"),grs::Body("Segment1.Link1"),grs::OrientationFrame("F1.Segment1.Link1"), link1Joint1FrameOrientationCoord);

        grs::Pose<KDL::Vector, KDL::Rotation> link1Joint1FramePose(link1Joint1FramePosition, link1Joint1FrameOrientation);
        
        //Joint2
        KDL::Vector link1Joint2FramePositionCoord = KDL::Vector(0, 0.35,0); //position of joint frame's origin
        grs::Position<KDL::Vector> link1Joint2FramePosition(grs::Point("j2.Segment1.Link1"),grs::Body("Segment1.Link1"),
                                                            grs::Point("f1.Segment1.Link1"), grs::Body("Segment1.Link1"), grs::OrientationFrame("F1.Segment1.Link1"), link1Joint2FramePositionCoord);
        KDL::Rotation link1Joint2FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> link1Joint2FrameOrientation(grs::OrientationFrame("J2.Segment1.Link1"),grs::Body("Segment1.Link1"),
                                                                    grs::OrientationFrame("F1.Segment1.Link1"),grs::Body("Segment1.Link1"),grs::OrientationFrame("F1.Segment1.Link1"), link1Joint2FrameOrientationCoord);

        grs::Pose<KDL::Vector, KDL::Rotation> link1Joint2FramePose(link1Joint2FramePosition, link1Joint2FrameOrientation);
        
        //Joint3
        KDL::Vector link1Joint3FramePositionCoord = KDL::Vector(0, 0.5,0); //position of joint frame's origin
        grs::Position<KDL::Vector> link1Joint3FramePosition(grs::Point("j3.Segment1.Link1"),grs::Body("Segment1.Link1"),
                                                            grs::Point("f1.Segment1.Link1"), grs::Body("Segment1.Link1"), grs::OrientationFrame("F1.Segment1.Link1"), link1Joint3FramePositionCoord);
        KDL::Rotation link1Joint3FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> link1Joint3FrameOrientation(grs::OrientationFrame("J3.Segment1.Link1"),grs::Body("Segment1.Link1"),
                                                                    grs::OrientationFrame("F1.Segment1.Link1"),grs::Body("Segment1.Link1"),grs::OrientationFrame("F1.Segment1.Link1"), link1Joint3FrameOrientationCoord);

        grs::Pose<KDL::Vector, KDL::Rotation> link1Joint3FramePose(link1Joint3FramePosition, link1Joint3FrameOrientation);

        typedef std::vector< kdle::AttachmentFrame<grs::Pose<KDL::Vector, KDL::Rotation> > > AttachmentFrames;
        AttachmentFrames frameList1;
        
        frameList1.push_back(kdle::createAttachmentFrame(link1Joint1FramePose, kdle::FrameType::JOINT));
        frameList1.push_back(kdle::createAttachmentFrame(link1Joint2FramePose, kdle::FrameType::JOINT));
        frameList1.push_back(kdle::createAttachmentFrame(link1Joint3FramePose, kdle::FrameType::JOINT));
        
    //LINK2 JOINT FRAMES
        //Joint1
        KDL::Vector link2Joint1FramePositionCoord = KDL::Vector(0.15,0,0); //position of joint frame's origin
        grs::Position<KDL::Vector> link2Joint1FramePosition(grs::Point("j1.Segment2.Link2"),grs::Body("Segment2.Link2"),
                                                            grs::Point("f2.Segment2.Link2"), grs::Body("Segment2.Link2"), grs::OrientationFrame("F2.Segment2.Link2"), link2Joint1FramePositionCoord);
        KDL::Rotation link2Joint1FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> link2Joint1FrameOrientation(grs::OrientationFrame("J1.Segment2.Link2"),grs::Body("Segment2.Link2"),
                                                                    grs::OrientationFrame("F2.Segment2.Link2"),grs::Body("Segment2.Link2"),grs::OrientationFrame("F2.Segment2.Link2"), link2Joint1FrameOrientationCoord);
        grs::Pose<KDL::Vector, KDL::Rotation> link2Joint1FramePose(link2Joint1FramePosition, link2Joint1FrameOrientation);
        
        //Joint2
        KDL::Vector link2Joint2FramePositionCoord = KDL::Vector(0.45,0,0); //position of joint frame's origin
        grs::Position<KDL::Vector> link2Joint2FramePosition(grs::Point("j2.Segment2.Link2"),grs::Body("Segment2.Link2"),
                                                            grs::Point("f2.Segment2.Link2"),grs::Body("Segment2.Link2"), grs::OrientationFrame("F2.Segment2.Link2"), link2Joint2FramePositionCoord);
        KDL::Rotation link2Joint2FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> link2Joint2FrameOrientation(grs::OrientationFrame("J2.Segment2.Link2"),grs::Body("Segment2.Link2"),
                                                                    grs::OrientationFrame("F2.Segment2.Link2"),grs::Body("Segment2.Link2"),grs::OrientationFrame("F2.Segment2.Link2"), link2Joint2FrameOrientationCoord);
        grs::Pose<KDL::Vector, KDL::Rotation> link2Joint2FramePose(link2Joint2FramePosition, link2Joint2FrameOrientation);
        

        //Joint3
        KDL::Vector link2Joint3FramePositionCoord = KDL::Vector(0.5,0,0); //position of joint frame's origin
        grs::Position<KDL::Vector> link2Joint3FramePosition(grs::Point("j3.Segment2.Link2"),grs::Body("Segment2.Link2"),
                                                            grs::Point("f2.Segment2.Link2"),grs::Body("Segment2.Link2"), grs::OrientationFrame("F2.Segment2.Link2"), link2Joint3FramePositionCoord);
        KDL::Rotation link2Joint3FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> link2Joint3FrameOrientation(grs::OrientationFrame("J3.Segment2.Link2"),grs::Body("Segment2.Link2"),
                                                                    grs::OrientationFrame("F2.Segment2.Link2"),grs::Body("Segment2.Link2"),grs::OrientationFrame("F2.Segment2.Link2"), link2Joint3FrameOrientationCoord);
        grs::Pose<KDL::Vector, KDL::Rotation> link2Joint3FramePose(link2Joint3FramePosition, link2Joint3FrameOrientation);

        typedef std::vector< kdle::AttachmentFrame<grs::Pose<KDL::Vector, KDL::Rotation> > > AttachmentFrames1;
        AttachmentFrames1 frameList2;
        
        
        frameList2.push_back(kdle::createAttachmentFrame(link2Joint1FramePose, kdle::FrameType::JOINT));
        frameList2.push_back(kdle::createAttachmentFrame(link2Joint2FramePose, kdle::FrameType::JOINT));
        frameList2.push_back(kdle::createAttachmentFrame(link2Joint3FramePose, kdle::FrameType::JOINT));
        
    //LINK3 JOINT FRAMES
        //Joint1
        KDL::Vector link3Joint1FramePositionCoord = KDL::Vector(0.10,0,0); //position of joint frame's origin
        grs::Position<KDL::Vector> link3Joint1FramePosition(grs::Point("j1.Segment3.Link3"),grs::Body("Segment3.Link3"),
                                                            grs::Point("f3.Segment3.Link3"),grs::Body("Segment3.Link3"),grs::OrientationFrame("F3.Segment3.Link3"), link3Joint1FramePositionCoord);
        KDL::Rotation link3Joint1FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> link3Joint1FrameOrientation(grs::OrientationFrame("J1.Segment3.Link3"),grs::Body("Segment3.Link3"),
                                                                    grs::OrientationFrame("F3.Segment3.Link3"),grs::Body("Segment3.Link3"),grs::OrientationFrame("F3.Segment3.Link3"), link3Joint1FrameOrientationCoord);
        grs::Pose<KDL::Vector, KDL::Rotation> link3Joint1FramePose(link3Joint1FramePosition, link3Joint1FrameOrientation);
               
        //Joint2
        KDL::Vector link3Joint2FramePositionCoord = KDL::Vector(0.35,0,0); //position of joint frame's origin
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
        KDL::Vector link4Joint2FramePositionCoord = KDL::Vector(0.0,0,0.15); //position of joint frame's origin
        grs::Position<KDL::Vector> link4Joint2FramePosition(grs::Point("j2.Segment4.Link4"),grs::Body("Segment4.Link4"),
                                                            grs::Point("f4.Segment4.Link4"),grs::Body("Segment4.Link4"),grs::OrientationFrame("F4.Segment4.Link4"), link4Joint2FramePositionCoord);
        KDL::Rotation link4Joint2FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> link4Joint2FrameOrientation(grs::OrientationFrame("J2.Segment4.Link4"),grs::Body("Segment4.Link4"),
                                                                    grs::OrientationFrame("F4.Segment4.Link4"),grs::Body("Segment4.Link4"),grs::OrientationFrame("F4.Segment4.Link4"), link4Joint2FrameOrientationCoord);
        grs::Pose<KDL::Vector, KDL::Rotation> link4Joint2FramePose(link4Joint2FramePosition, link4Joint2FrameOrientation);
        
        
        AttachmentFrames1 frameList4;
        frameList4.push_back(kdle::createAttachmentFrame(link4Joint1FramePose, kdle::FrameType::JOINT));
        frameList4.push_back(kdle::createAttachmentFrame(link4Joint2FramePose, kdle::FrameType::JOINT));
        
        
    //LINK5 JOINT FRAMES
        //Joint1
        KDL::Vector link5Joint1FramePositionCoord = KDL::Vector(0.0,0,0.25); //position of joint frame's origin
        grs::Position<KDL::Vector> link5Joint1FramePosition(grs::Point("j1.Segment5.Link5"),grs::Body("Segment5.Link5"),
                                                            grs::Point("f5.Segment5.Link5"),grs::Body("Segment5.Link5"),grs::OrientationFrame("F5.Segment5.Link5"), link5Joint1FramePositionCoord);
        KDL::Rotation link5Joint1FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> link5Joint1FrameOrientation(grs::OrientationFrame("J1.Segment5.Link5"),grs::Body("Segment5.Link5"),
                                                                    grs::OrientationFrame("F5.Segment5.Link5"),grs::Body("Segment5.Link5"),grs::OrientationFrame("F5.Segment5.Link5"), link5Joint1FrameOrientationCoord);
        grs::Pose<KDL::Vector, KDL::Rotation> link5Joint1FramePose(link5Joint1FramePosition, link5Joint1FrameOrientation);
        
        //Joint2
        KDL::Vector link5Joint2FramePositionCoord = KDL::Vector(0.0,0,0.55); //position of joint frame's origin
        grs::Position<KDL::Vector> link5Joint2FramePosition(grs::Point("j2.Segment5.Link5"),grs::Body("Segment5.Link5"),
                                                            grs::Point("f5.Segment5.Link5"),grs::Body("Segment5.Link5"),grs::OrientationFrame("F5.Segment5.Link5"), link5Joint2FramePositionCoord);
        KDL::Rotation link5Joint2FrameOrientationCoord = KDL::Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> link5Joint2FrameOrientation(grs::OrientationFrame("J2.Segment5.Link5"),grs::Body("Segment5.Link5"),
                                                                    grs::OrientationFrame("F5.Segment5.Link5"),grs::Body("Segment5.Link5"),grs::OrientationFrame("F5.Segment5.Link5"), link5Joint2FrameOrientationCoord);
        grs::Pose<KDL::Vector, KDL::Rotation> link5Joint2FramePose(link5Joint2FramePosition, link5Joint2FrameOrientation);
        
        AttachmentFrames1 frameList5;
        frameList5.push_back(kdle::createAttachmentFrame(link5Joint1FramePose, kdle::FrameType::JOINT));
        frameList5.push_back(kdle::createAttachmentFrame(link5Joint2FramePose, kdle::FrameType::JOINT));

        //Segment specification with two argument Pose
        //attaching frames at construction time
        kdle::Segment< grs::Pose<KDL::Vector, KDL::Rotation> > segment1_grs("Segment1", link1, frameList1);
        kdle::Segment< grs::Pose<KDL::Vector, KDL::Rotation> > segment2_grs("Segment2", link2, frameList2);
        kdle::Segment< grs::Pose<KDL::Vector, KDL::Rotation> > segment3_grs("Segment3", link3, frameList3);
        kdle::Segment< grs::Pose<KDL::Vector, KDL::Rotation> > segment4_grs("Segment4", link4, frameList4);
        kdle::Segment< grs::Pose<KDL::Vector, KDL::Rotation> > segment5_grs("Segment5", link5, frameList5);

        kdle::JointProperties joint1_props, joint2_props, joint3_props, joint4_props;
        joint1_props = std::make_tuple(kdle::JointTypes::REVOLUTE_X, 0.02, 150.0, -140.0);
        joint2_props = std::make_tuple(kdle::JointTypes::REVOLUTE_Z, 0.02, 150.0, -140.0);
        joint3_props = std::make_tuple(kdle::JointTypes::REVOLUTE_X, 0.025, 145.0, -135.0);
        joint4_props = std::make_tuple(kdle::JointTypes::REVOLUTE_Z, 0.02, 150.0, -140.0);
        
        kdle::Joint< grs::Pose<KDL::Vector, KDL::Rotation> > joint1("Joint1", segment1_grs.getAttachmentFrames()[2], segment1_grs, segment5_grs.getAttachmentFrames()[0], segment5_grs, joint1_props);
        kdle::Joint< grs::Pose<KDL::Vector, KDL::Rotation> > joint2("Joint2", segment2_grs.getAttachmentFrames()[0], segment2_grs, segment1_grs.getAttachmentFrames()[1], segment1_grs, joint1_props);
        kdle::Joint< grs::Pose<KDL::Vector, KDL::Rotation> > joint3("Joint3", segment4_grs.getAttachmentFrames()[0], segment4_grs, segment2_grs.getAttachmentFrames()[2], segment2_grs, joint3_props);
        kdle::Joint< grs::Pose<KDL::Vector, KDL::Rotation> > joint4("Joint4", segment3_grs.getAttachmentFrames()[0], segment3_grs, segment2_grs.getAttachmentFrames()[1], segment2_grs,  joint4_props);
        
        
        kdle::TransmissionProperties trans_props = std::make_tuple(0.2,0.2,0.2);
        kdle::make_transmission(joint1,trans_props);
        kdle::make_transmission(joint2,trans_props);
        kdle::make_transmission(joint3,trans_props);
        kdle::make_transmission(joint4,trans_props);
        
        grs::Pose<KDL::Vector, KDL::Rotation> currentJointPose, currentJointPose1, currentJointPose2, currentJointPose3;
        grs::Twist<KDL::Vector, KDL::Vector> currentJointTwist, twisttemp;
        grs::Twist<KDL::Twist> twisttemp1;
        
        //joint value is of type vector whose size changes according to the joint's DoF
        std::vector<double> jointvalue(1,M_PI/4.0);
        std::vector<double> jointtwistvalue(1,0.855);
        std::vector<double> jointtwistvalue1(1,2.25);
        std::vector<double> jointvalue1(1,-M_PI/6.0);
        
//        joint1.getPoseOfJointFrames(jointvalue, currentJointPose );
//        joint1.getPoseCurrentDistalToPredecessorDistal(jointvalue, currentJointPose1 );
//        joint1.getPoseCurrentDistalToPredecessorRefJointFrame(jointvalue, currentJointPose1 );
////        
        joint2.getPoseOfJointFrames(jointvalue, currentJointPose );
        joint2.getPoseCurrentDistalToPredecessorDistal(jointvalue, currentJointPose1 );
        joint2.getPoseCurrentDistalToPredecessorRefJointFrame(jointvalue, currentJointPose1 );
////        
//        joint3.getPoseOfJointFrames(jointvalue1, currentJointPose1 );
//        joint3.getPoseCurrentDistalToPredecessorDistal(jointvalue1, currentJointPose);
//        joint3.getPoseCurrentDistalToPredecessorRefJointFrame(jointvalue1, currentJointPose );
//        
//        joint4.getPoseOfJointFrames(jointvalue1, currentJointPose1 );
//        joint4.getPoseCurrentDistalToPredecessorDistal(jointvalue1, currentJointPose);
//        joint4.getPoseCurrentDistalToPredecessorRefJointFrame(jointvalue1, currentJointPose);
//
//        joint1.getTwistOfJointFrames(jointtwistvalue, twisttemp);
//        joint1.getTwistCurrentDistalToPredecessorJointFrame(jointvalue, jointtwistvalue, twisttemp);
//        
        joint2.getTwistOfJointFrames(jointtwistvalue, twisttemp);
        joint2.getTwistCurrentDistalToPredecessorJointFrame(jointvalue, jointtwistvalue, twisttemp);
//        
//        joint3.getTwistOfJointFrames(jointtwistvalue1 ,twisttemp);
//        joint3.getTwistCurrentDistalToPredecessorJointFrame(jointvalue1, jointtwistvalue1, twisttemp);
//
//        joint4.getTwistOfJointFrames(jointtwistvalue, twisttemp);
//        joint4.getTwistCurrentDistalToPredecessorJointFrame(jointvalue, jointtwistvalue, twisttemp);
        
        typedef std::vector< kdle::Joint< grs::Pose<KDL::Vector, KDL::Rotation> > > JointList;
        typedef std::map<std::string, kdle::Joint< grs::Pose<KDL::Vector, KDL::Rotation> > > JointListMap;
        JointList jointlist, jointlist1;
        jointlist.push_back(joint1);
        jointlist.push_back(joint2);
        jointlist.push_back(joint3);
        jointlist.push_back(joint4);
        
        jointlist1.push_back(joint3);
        jointlist1.push_back(joint1);
        jointlist1.push_back(joint2);
        jointlist1.push_back(joint4);
        
//        kdle::KinematicChain< grs::Pose<KDL::Vector, KDL::Rotation>, JointList > mychain("MyKinematicChain", jointlist);
        kdle::KinematicChain< grs::Pose<KDL::Vector, KDL::Rotation>, JointList > mychain("MyKinematicChain1", jointlist1);
        std::cout << "mychain.size " << mychain.getNrOfSegments() << std::endl << std::endl << std::endl << std::endl;
       
        
    //~SEGMENT METADATA
    //Computational operation
        typedef kdle::Composite< kdle::transform<kdle::grs_iterator, kdle::twist>, kdle::transform<kdle::grs_iterator, kdle::pose> > compositeOperationType;
        kdle::transform<kdle::grs_iterator, kdle::pose> forwardPoseOperation;
        kdle::transform<kdle::grs_iterator, kdle::twist> forwardTwistOperation;
        compositeOperationType forwardKinematics = compose(forwardTwistOperation, forwardPoseOperation );
    //Traversal policy
        kdle::DFSPolicy< kdle::KinematicChain< grs::Pose<KDL::Vector, KDL::Rotation> > > policy;
        std::vector<kdle::ComputationalState<grs::Pose<KDL::Vector, KDL::Rotation>, grs::Twist<KDL::Vector, KDL::Vector>, kdle::StateSpaceType::JointSpace> > jstate;
        jstate.resize(mychain.getNrOfJoints());
        jstate[0].q.push_back(KDL::PI/4.0);
        jstate[0].qdot.push_back(0.8555);
        jstate[1].q.push_back(-KDL::PI/6.0);
        jstate[1].qdot.push_back(2.25);
        jstate[2].q.push_back(KDL::PI / 4.0);
        jstate[2].qdot.push_back(-0.2);
        jstate[3].q.push_back(KDL::PI / 4.0);
        jstate[3].qdot.push_back(-0.2);
        
        std::vector<kdle::ComputationalState<grs::Pose<KDL::Vector, KDL::Rotation>, grs::Twist<KDL::Vector, KDL::Vector>, kdle::StateSpaceType::CartesianSpace> > lstate;
        lstate.resize(mychain.getNrOfJoints());
        std::vector<kdle::ComputationalState<grs::Pose<KDL::Vector, KDL::Rotation>, grs::Twist<KDL::Vector, KDL::Vector>, kdle::StateSpaceType::CartesianSpace> > lstate2;
        lstate2.resize(mychain.getNrOfJoints());
    //Traversal operation
        kdle::traverseGraph(mychain, forwardKinematics, policy)(jstate, lstate, lstate2);
    return 0;
}

