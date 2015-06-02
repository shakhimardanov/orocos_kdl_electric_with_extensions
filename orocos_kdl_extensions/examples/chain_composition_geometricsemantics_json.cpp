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
//#define VERBOSE_WALK 

#include <graphviz/gvc.h>
#include <graphviz/graph.h>
#include <Variant/Variant.h>
#include <Variant/Schema.h>
#include <Variant/SchemaLoader.h>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl_extensions/functionalcomputation_kdl.hpp>

using namespace std;
using libvariant::Variant;

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


void walkJSONTree(Variant const& inputData, std::vector<SemanticData>& semanticData)
{
    if (inputData.IsMap())
    {
//        std::cout << "TOP: IT IS A MAP" << std::endl;
//        printf("Size: %d \n",inputData.Size());
//        std::cout  << std::endl;
        for (Variant::ConstMapIterator i=inputData.MapBegin(); i != inputData.MapEnd(); ++i)
        {
            if(i->first == "@kinematics")
            {
                std::cout << "Value: "<<  std::endl<< std::endl;
                if(i->first == "@kinematictype")
                {
                    std::cout << "Value: "<< inputData.Get(i->first).AsString()<< std::endl<< std::endl;
                }
            }
            
            if(i->first == "@geometry")
            {
                if(i->first == "@geomtype")
                {
                    std::cout << "Value: "<< inputData.Get(i->first).AsString()<< std::endl<< std::endl;
                }
            }
            if( (i->second.IsMap())||(i->second.IsList()) )
            {
//                printf("Type: %d \n", i->second.GetType());
//                std::cout << "Value: " << std::endl<< std::endl;
                walkJSONTree(i->second, semanticData);
            }
            else if(i->second.IsString())
            {
              
//                std::cout << "BOTTOM: IT IS A STRING" << std::endl;
//                printf("Key: %s \n", i->first.c_str());
//                printf("Type: %d \n", i->second.GetType());
//                std::cout << "Value: "<< inputData.Get(i->first).AsString()<< std::endl<< std::endl;

            }
            else if(i->second.IsFloat())
            {
//                std::cout << "BOTTOM: IT IS A FLOAT" << std::endl;
//                printf("Key: %s \n", i->first.c_str());
//                printf("Type: %d \n", i->second.GetType());
//                std::cout << "Value: "<< i->second.AsString() << std::endl<< std::endl;
            }
        } 
    }
    
    if(inputData.IsList())
    {
//        std::cout << "TOP: IT IS A LIST" << std::endl;
//        printf("Size: %d \n",inputData.Size());
        for (Variant::ConstListIterator j = inputData.ListBegin(); j != inputData.ListEnd(); ++j)
        {
            
//            printf("Type: %d \n", j->GetType());
//            std::cout  << std::endl;
            walkJSONTree(*j, semanticData);
        } 

    }
    return;
}

void walkJSONTree(Variant const& inputData,  Agnode_t* node, Agedge_t* edgeVector, Agraph_t *g, int& nodecounter)
{
    if (inputData.IsMap())
    {
        Agnode_t* parentnode;
        parentnode = node;
        nodecounter++;
        
#ifdef VERBOSE_WALK
        std::cout << "PLOT: TOP: IT IS A MAP" << std::endl;
        printf("Size: %d \n",inputData.Size());
        std::cout  << std::endl;
#endif
        for (Variant::ConstMapIterator i=inputData.MapBegin(); i != inputData.MapEnd(); ++i)
        {   

            if(i->second.IsMap())
            {
                
#ifdef VERBOSE_WALK
                std::cout << "BOTTOM: IT IS A MAP" << std::endl;
                printf("Key: %s \n", i->first.c_str());
                printf("Type: %d \n", i->second.GetType());
                std::cout << "Value: " << std::endl<< std::endl;
#endif          
                nodecounter++;
                std::string tag("node");
                std::ostringstream convert;
                convert << nodecounter;
                tag.append("-");
                tag.append(convert.str());
                tag.append(":");
                tag.append(i->first);
                
                Agnode_t* currentnode = agnode( g, const_cast<char*>(tag.c_str()) );
                agsafeset(currentnode, "color", "green", "");
                agsafeset(currentnode, "shape", "circle", "");
                edgeVector = agedge(g, parentnode , currentnode);
                walkJSONTree(i->second, currentnode, edgeVector, g, nodecounter);
            }
            else if(i->second.IsList())
            {
                
#ifdef VERBOSE_WALK
                std::cout << "BOTTOM: IT IS A LIST" << std::endl;
                printf("Key: %s \n", i->first.c_str());
                printf("Type: %d \n", i->second.GetType());
                std::cout << "Value: " << std::endl<< std::endl;
#endif
                nodecounter++;
                std::string tag("node");
                std::ostringstream convert;
                convert << nodecounter;
                tag.append("-");
                tag.append(convert.str());
                tag.append(":");
                tag.append(i->first);
                
                Agnode_t* currentnode = agnode( g, const_cast<char*>(tag.c_str()) );
                agsafeset(currentnode, "color", "blue", "");
                agsafeset(currentnode, "shape", "oval", "");
                edgeVector= agedge(g, parentnode , currentnode);
                walkJSONTree(i->second, currentnode, edgeVector, g, nodecounter);
            }
            else if(i->second.IsString())
            {
                
#ifdef VERBOSE_WALK
                std::cout << "BOTTOM: IT IS A STRING" << std::endl;
                printf("Key: %s \n", i->first.c_str());
                printf("Type: %d \n", i->second.GetType());
                std::cout << "Value: " << std::endl<< std::endl;
#endif               
                nodecounter++;
                std::string tag("node");
                std::ostringstream convert;
                convert << nodecounter;
                tag.append("-");
                tag.append(convert.str());
                tag.append(":");
                tag.append(i->first);                
                tag.append(":");
                tag.append(i->second.AsString());
                
                //fill in edge vector by iterating over joints in the tree
                Agnode_t* currentnode = agnode( g, const_cast<char*>(tag.c_str()) );
                agsafeset(currentnode, "color", "red", "");
                agsafeset(currentnode, "shape", "box", "");

                edgeVector= agedge(g, parentnode , currentnode);
//                agsafeset(edgeVector.back(), "label",  const_cast<char*>(i->second.AsString().c_str()), "");

            }
            else if(i->second.IsFloat())
            {

#ifdef VERBOSE_WALK
                std::cout << "BOTTOM: IT IS A FLOAT" << std::endl;
                printf("Key: %s \n", i->first.c_str());
                printf("Type: %d \n", i->second.GetType());
                std::cout << "Value: " << std::endl<< std::endl;
#endif                
                nodecounter++;
                std::string tag("node");
                std::ostringstream convert;
                convert << nodecounter;
                tag.append("-");
                tag.append(convert.str());
                tag.append(":");
                tag.append(i->first);                
                tag.append(":");
                tag.append(i->second.AsString());
                
                Agnode_t* currentnode = agnode( g, const_cast<char*>(tag.c_str()) );
                agsafeset(currentnode, "color", "red", "");
                agsafeset(currentnode, "shape", "box", "");

                edgeVector= agedge(g, parentnode , currentnode);
            }
            
        } 
    }
    
    else if(inputData.IsList())
    {
        nodecounter++;
        Agnode_t* parentnode;
        parentnode = node;

#ifdef VERBOSE_WALK
        std::cout << "TOP: IT IS A LIST" << std::endl;
        printf("Size: %d \n",inputData.Size());
#endif
        for (Variant::ConstListIterator j = inputData.ListBegin(); j != inputData.ListEnd(); ++j)
        {
            if (j->IsMap())
            {
                //label the child node
                nodecounter++;
                std::string tag("node");
                std::ostringstream convert;
                convert << nodecounter;
                tag.append("-");
                tag.append(convert.str());
                //connect the parent and the child nodes
                Agnode_t* currentnode = agnode( g, const_cast<char*>(tag.c_str()) );
                agsafeset(currentnode, "color", "green", "");
                agsafeset(currentnode, "shape", "circle", "");
                edgeVector= agedge(g, parentnode , currentnode);
                //go to top of the function
                walkJSONTree(*j, currentnode, edgeVector, g, nodecounter);
            }
            //similar if clauses might be required for homogeneous arrays (i.e. the elements are of the same type)

        }
    }
    
    return;
}

void drawTree(Variant const& inputData)
{
    //graphviz stuff
    /****************************************/
    Agraph_t *g;
    GVC_t *gvc;

    /* set up a graphviz context */
    gvc = gvContext();
    /* Create a simple digraph */
    g = agopen("robot-structure", AGDIGRAPH);

    //create root node
    Agnode_t* rootnode = agnode( g, "/" );
    //create vector to hold edges
    Agedge_t* edgeVector;
    int counter = 0;
    walkJSONTree(inputData, rootnode, edgeVector, g, counter);
    
   /* Compute a layout using layout engine from command line args */
    
    gvLayout(gvc, g, "dot");

    /* Write the graph according to -T and -o options */
    //gvRenderJobs(gvc, g);
    gvRenderFilename(gvc, g, "pdf", "semantics-tree.pdf");

    /* Free layout data */
    gvFreeLayout(gvc, g);

    /* Free graph structures */
    agclose(g);

    gvFreeContext(gvc);
    /* close output file, free context, and return number of errors */
    return;
}

bool createTree(std::string const& inputModelFile, std::string const& inputSchemaFile, std::vector<SemanticData>& semanticData, bool const& plotOff=true)
{
    Variant v = libvariant::DeserializeJSONFile(inputModelFile.c_str());
    libvariant::AdvSchemaLoader loader;                                                                    
    libvariant::SchemaResult result = libvariant::SchemaValidate(std::string("file://").append(inputSchemaFile).c_str(), v, &loader);
    
    if(!plotOff)
        drawTree(v);
    
    if (!result.Error())
    {
       walkJSONTree(v, semanticData);
       return true;
    }
    else
    {
        cout << result << endl;
        return false;
    }
}

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

void initSegmentFrameGeometry(SegmentGeometricData *segmentgeoms, const unsigned int size)
{
    segmentgeoms[0].link_root_position_coord = KDL::Vector(0, 0, 0);
    segmentgeoms[0].link_root_orientation_coord = KDL::Rotation::RotZ(0.0);
    segmentgeoms[0].link_tip_position_coord = KDL::Vector(0.5, 0.0, 0.0);
    segmentgeoms[0].link_tip_orientation_coord = KDL::Rotation::Identity();

    segmentgeoms[1].link_root_position_coord = KDL::Vector(0.1, 0, 0);
    segmentgeoms[1].link_root_orientation_coord = KDL::Rotation::RotZ(0.0);
    segmentgeoms[1].link_tip_position_coord = KDL::Vector(0.5, 0.0, 0.0);
    segmentgeoms[1].link_tip_orientation_coord = KDL::Rotation::Identity();

    segmentgeoms[2].link_root_position_coord = KDL::Vector(0.2, 0, 0);
    segmentgeoms[2].link_root_orientation_coord = KDL::Rotation::RotZ(0.0);
    segmentgeoms[2].link_tip_position_coord = KDL::Vector(0.5, 0.0, 0.0);
    segmentgeoms[2].link_tip_orientation_coord = KDL::Rotation::Identity();

    segmentgeoms[3].link_root_position_coord = KDL::Vector(0, 0, 0.15);
    segmentgeoms[3].link_root_orientation_coord = KDL::Rotation::RotZ(0.0);
    segmentgeoms[3].link_tip_position_coord = KDL::Vector(0.0, 0.0, 0.65);
    segmentgeoms[3].link_tip_orientation_coord = KDL::Rotation::Identity();

    segmentgeoms[4].link_root_position_coord = KDL::Vector(0, 0, 0);
    segmentgeoms[4].link_root_orientation_coord = KDL::Rotation::RotZ(0.0);
    segmentgeoms[4].link_tip_position_coord = KDL::Vector(0.0, 0.0, -0.75);
    segmentgeoms[4].link_tip_orientation_coord = KDL::Rotation::Identity();
}

int main(int argc, char** argv)
{
    
    std::string filename("json-models/input-kinematics-dsl.json");
    std::string schemaname("/home/azamat/programming/ros-electric/orocos_kinematics_dynamics/orocos_kdl_extensions/json-models/kinematics-dsl.json");
 
    std::vector<SemanticData> semanticData;
    if(!createTree(filename, schemaname, semanticData, false))
    {
        return 1;
    }
    
    unsigned int numberOfSegments = 5;
    unsigned int numberOfJointFramesPerSegment = 3;
    unsigned int numberOfSegmentFrames = 2 * numberOfSegments + 1;
    unsigned int numberOfJointFrames = numberOfJointFramesPerSegment*numberOfSegments;

    SemanticData segmentframesemantcs[numberOfSegmentFrames];
    initSegmentFrameSemantics(segmentframesemantcs, numberOfSegmentFrames);
    SegmentGeometricData segmentframecoord[numberOfSegmentFrames];
    initSegmentFrameGeometry(segmentframecoord, numberOfSegmentFrames);
    
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

