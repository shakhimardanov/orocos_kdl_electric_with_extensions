/* 
 * File:   json_to_semantics_parser.hpp
 * Author: azamat
 *
 * Created on June 10, 2015, 1:19 PM
 */

#ifndef JSON_TO_SEMANTICS_PARSER_HPP
#define	JSON_TO_SEMANTICS_PARSER_HPP

#include <graphviz/gvc.h>
#include <graphviz/graph.h>
#include <Variant/Variant.h>
#include <Variant/Schema.h>
#include <Variant/SchemaLoader.h>
#include <kdl_extensions/geometric_semantics_kdl.hpp>
#include <iostream>
#include <Eigen/Core>
#include <kdl/frames_io.hpp>
#include <kdl_extensions/chain_geometric_primitives.hpp>

using libvariant::Variant;
using std::map; 
using std::string;
using namespace std;

template <typename PoseT>
bool jsonToPose(Variant const& inputData, PoseT& geometricPrimitive)
{
   
    std::string geomtype = inputData.At("@geomtype").AsString();

    grs::Point refPoint, tarPoint;
    grs::Body refBody, tarBody;
    grs::OrientationFrame refFrame, tarFrame, coordFrame;
    KDL::Frame homogenTransform;
    
    Variant const semantics = inputData.At("semantics");
    for (Variant::ConstMapIterator i=semantics.MapBegin(); i != semantics.MapEnd(); ++i)
    {
        if(i->first == "target")
        {
            std::string pointname = i->second.At("point").At("@semantic_primitive").At("name").AsString().append(inputData.At("name").AsString());
            tarPoint = grs::Point(pointname);
            std::string framename = i->second.At("frame").At("@semantic_primitive").At("name").AsString().append(inputData.At("name").AsString()); 
            tarFrame = grs::OrientationFrame(framename);
            std::string bodyname = i->second.At("body").At("@semantic_primitive").At("name").AsString().append(inputData.At("name").AsString());
            tarBody = grs::Body(bodyname);

        }
        if(i->first == "reference")
        {
            std::string pointname = i->second.At("point").At("@semantic_primitive").At("name").AsString().append(inputData.At("name").AsString());
            refPoint = grs::Point(pointname);
            std::string framename = i->second.At("frame").At("@semantic_primitive").At("name").AsString().append(inputData.At("name").AsString());
            refFrame = grs::OrientationFrame(framename);
            std::string bodyname = i->second.At("body").At("@semantic_primitive").At("name").AsString().append(inputData.At("name").AsString());
            refBody = grs::Body(bodyname);
        }
        if(i->first == "coordinateFrame")
        {
            std::string name = i->second.At("@semantic_primitive").At("name").AsString().append(inputData.At("name").AsString());
            coordFrame = grs::OrientationFrame(name);
        }
    }

    Variant const coordinates = inputData.At("coordinates");

    for (Variant::ConstMapIterator i=coordinates.MapBegin(); i != coordinates.MapEnd(); ++i)
    {
        if(i->first == "rows")
        {
            Eigen::Matrix<double, 4, 4 > Matrix4d;

            unsigned int matrixrow(0);
            for (Variant::ConstListIterator k = i->second.ListBegin(); k != i->second.ListEnd(); ++k)
            {
                unsigned int matrixcolumn(0);
                for (Variant::ConstListIterator j = k->ListBegin(); j != k->ListEnd(); ++j)
                {

                    Matrix4d(matrixrow,matrixcolumn) = j->AsDouble();
                    matrixcolumn++;
                }
                matrixrow++;
            }

            for(unsigned int i=0; i<3; i++)
            {
                homogenTransform.p.data[i] = Matrix4d(i,3);
            }

            unsigned int k(0);
            for(unsigned int i=0; i<3; i++)
            {
                for(unsigned int j=0; j<3; j++)
                {
                    homogenTransform.M.data[k] = Matrix4d(i,j);
                    k++;
                }
            }
        }
    }

    geometricPrimitive = grs::Pose<KDL::Frame>(grs::PoseCoordinatesSemantics(tarPoint, tarFrame,tarBody, refPoint, refFrame,refBody,coordFrame),homogenTransform);

    return true;
}

template <typename PoseT>
bool jsonToSegment(Variant const& inputData, kdle::Segment<PoseT>& kinematicsPrimitive)
{
    PoseT rootFrame, tipFrame, jointFrame;
    
    kdle::Link< PoseT > link;
    std::vector< kdle::AttachmentFrame<PoseT> > frameList1;
    
    std::cout << inputData.At("@kinematicstype").AsString() << std::endl;

    Variant const jointsemantics = inputData.At("link").At("@kinematics");
    for (Variant::ConstMapIterator i=jointsemantics.MapBegin(); i != jointsemantics.MapEnd(); ++i)
    {
        if(i->first == "rootFrame")
        {
            jsonToPose(i->second.At("@geometry"), rootFrame);
            std::cout << "rootFrame= " << rootFrame << std::endl;

        }
        if(i->first == "tipFrame")
        {
            jsonToPose(i->second.At("@geometry"), tipFrame);
            std::cout << "tipFrame= " << tipFrame << std::endl;
        }
    }

    link = kdle::Link<PoseT> (inputData.At("link").At("@kinematics").At("name").AsString(), rootFrame, tipFrame);

    Variant const attachmentframes = inputData.At("framelist");
    for (Variant::ConstListIterator k=attachmentframes.ListBegin(); k != attachmentframes.ListEnd(); ++k)
    {
        jsonToPose (k->At("frame").At("@geometry"), jointFrame);
        std::cout <<"Joint pose= " << jointFrame << std::endl;
        frameList1.push_back( kdle::createAttachmentFrame(jointFrame, kdle::FrameType::JOINT));
    }
    kinematicsPrimitive = kdle::Segment<PoseT>(inputData.At("name").AsString(),link,frameList1);  
    return true;
}


template <typename PoseT>
bool jsonToJoint(Variant const& inputData, std::map<std::string, PoseT>& segmentList, kdle::Joint<PoseT>& kinematicsPrimitive)
{

    std::cout << inputData.At("@kinematicstype").AsString() << std::endl;
      
    Variant const jointsemantics = inputData.At("@jointtype");
    for (Variant::ConstMapIterator i=jointsemantics.MapBegin(); i != jointsemantics.MapEnd(); ++i)
    {
        if(i->first == "@constraint")
        {

        }
        if(i->first == "target")
        {
            std::string jointname= i->second.At("frameid").At("name").AsString();
            std::string jointid= i->second.At("frameid").At("id").AsString();
            std::cout << "targetframename= " << jointname << std::endl;

        }
        if(i->first == "reference")
        {
            std::string jointname= i->second.At("frameid").At("name").AsString();
            std::string jointid= i->second.At("frameid").At("id").AsString();
            std::cout << "referenceframename= " << jointname << std::endl;
        }
    }

    kinematicsPrimitive = kdle::Joint<PoseT>();  
    return true;
}

template <typename PoseT>
bool jsonToCppModel(Variant const& inputData, kdle::KinematicChain<PoseT>& achain)
{    
    if(inputData.IsList())
    {
        #ifdef VERBOSE_CHECK_KDLE
            std::cout << "TOP: IT IS A LIST" << std::endl;
            printf("Number of Kinematic structures: %d \n",inputData.Size());
        #endif  
        
        std::vector< kdle::Segment<PoseT> > segments;
        std::map<std::string, PoseT> segmentList;
        
        for (Variant::ConstListIterator j = inputData.ListBegin(); j != inputData.ListEnd(); ++j)
        {
            if (j->IsMap())
            {
                for (Variant::ConstMapIterator i=j->MapBegin(); i != j->MapEnd(); ++i)
                {
                    if( i->first == "@kinematics")
                    {
                        if(i->second.At("@kinematicstype").AsString() =="Segment")
                        {
                            kdle::Segment<grs::Pose<KDL::Frame> > segment;
                            jsonToSegment(i->second, segment);
                            segments.push_back(segment);
                        }
                        else if(i->second.At("@kinematicstype").AsString() =="Joint")
                        {
                            kdle::Joint<PoseT> joint;
                            jsonToJoint(i->second, segmentList, joint);
                        }
                    }
                } 
            }
        } 

    }
    return true;
}

void walkJSONTree(Variant const& inputData,  Agnode_t* node, Agedge_t* edgeVector, Agraph_t *g, int& nodecounter)
{
    if (inputData.IsMap())
    {
        Agnode_t* parentnode;
        parentnode = node;
        nodecounter++;
        
#ifdef VERBOSE_CHECK_KDLE
        std::cout << "PLOT: TOP: IT IS A MAP" << std::endl;
        printf("Size: %d \n",inputData.Size());
        std::cout  << std::endl;
#endif
        for (Variant::ConstMapIterator i=inputData.MapBegin(); i != inputData.MapEnd(); ++i)
        {   

            if(i->second.IsMap())
            {
                
#ifdef VERBOSE_CHECK_KDLE
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
                agsafeset(currentnode, "shape", "oval", "");
                edgeVector = agedge(g, parentnode , currentnode);
                walkJSONTree(i->second, currentnode, edgeVector, g, nodecounter);
            }
            else if(i->second.IsList())
            {
                
#ifdef VERBOSE_CHECK_KDLE
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
                
#ifdef VERBOSE_CHECK_KDLE
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
                agsafeset(currentnode, "shape", "oval", "");

                edgeVector= agedge(g, parentnode , currentnode);
//                agsafeset(edgeVector.back(), "label",  const_cast<char*>(i->second.AsString().c_str()), "");

            }
            else if(i->second.IsFloat())
            {

#ifdef VERBOSE_CHECK_KDLE
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

#ifdef VERBOSE_CHECK_KDLE
        std::cout << "PLOT: TOP: IT IS A LIST" << std::endl;
        printf("Size: %d \n",inputData.Size());
#endif
        for (Variant::ConstListIterator j = inputData.ListBegin(); j != inputData.ListEnd(); ++j)
        {
            if (j->IsMap() || j->IsList())
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
                agsafeset(currentnode, "shape", "oval", "");
                edgeVector= agedge(g, parentnode , currentnode);
                //go to top of the function
                walkJSONTree(*j, currentnode, edgeVector, g, nodecounter);
            }
            else if(j->IsFloat())
            {

#ifdef VERBOSE_CHECK_KDLE
                std::cout << "LIST BOTTOM: IT IS A FLOAT" << std::endl;
                printf("Type: %d \n", j->GetType());
                std::cout << "Value: " << j->AsNumber<float>() << std::endl<< std::endl;
#endif                
                nodecounter++;
                std::string tag("node");
                tag.append("-");
                tag.append(static_cast<std::ostringstream&>(std::ostringstream() << nodecounter).str());
                tag.append(":");
                tag.append(static_cast<std::ostringstream&>(std::ostringstream() << j->AsNumber<float>()).str());
                
                Agnode_t* currentnode = agnode( g, const_cast<char*>(tag.c_str()) );
                agsafeset(currentnode, "color", "red", "");
                agsafeset(currentnode, "shape", "box", "");

                edgeVector= agedge(g, parentnode , currentnode);
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

template <typename PoseT>
bool createTree(std::string const& inputModelFile, std::string const& inputSchemaFile, kdle::KinematicChain< PoseT >& achain, bool const& plotOff=true)
{
    Variant jsonData = libvariant::DeserializeJSONFile(inputModelFile.c_str());
    libvariant::AdvSchemaLoader loader;                                                                    
    libvariant::SchemaResult result = libvariant::SchemaValidate(std::string("file://").append(inputSchemaFile).c_str(), jsonData, &loader);
    
    if(!plotOff)
        drawTree(jsonData);
    
    if (!result.Error())
    {
        
        jsonToCppModel(jsonData, achain);
        return true;
    }
    else
    {
        cout << result << endl;
        return false;
    }
}



////this function should be made a template which returns a kdle geometry (Pose, Twist)
////based on the value of @geomtype
//grs::Pose<KDL::Frame>  jsonToPose(Variant const& inputData)
//{
//    #ifdef VERBOSE_CHECK_KDLE
//        std::string geomname = inputData.At("name").AsString();
//        std::cout << geomname << std::endl;
//        std::string geomid = inputData.At("id").AsString();
//        std::cout << geomid << std::endl;
//        std::string dsltype = inputData.At("@dsltype").AsString();
//        std::cout << dsltype << std::endl;
//        std::cout << geomtype << std::endl;
//    #endif
//    std::string geomtype = inputData.At("@geomtype").AsString();
//
//    grs::Point refPoint, tarPoint;
//    grs::Body refBody, tarBody;
//    grs::OrientationFrame refFrame, tarFrame, coordFrame;
//    KDL::Frame T;
//    
//    if(geomtype == "Position")
//    {
//        Variant const semantics = inputData.At("semantics");
//        for (Variant::ConstMapIterator i=semantics.MapBegin(); i != semantics.MapEnd(); ++i)
//        {
//            if(i->first == "target")
//            {
//                //point name is concatenated from its own name and the geometry's name it belongs.
//                std::string pointname = i->second.At("point").At("@semantic_primitive").At("name").AsString().append(inputData.At("name").AsString());
//                tarPoint = grs::Point(pointname);
//                //body name is concatenated from its own name and the geometry's name it belongs.
//                std::string bodyname = i->second.At("body").At("@semantic_primitive").At("name").AsString().append(inputData.At("name").AsString());
//                tarBody = grs::Body(bodyname);
//                
//            }
//            if(i->first == "reference")
//            {
//                std::string pointname = i->second.At("point").At("@semantic_primitive").At("name").AsString().append(inputData.At("name").AsString());
//                refPoint = grs::Point(pointname);
//                std::string bodyname = i->second.At("body").At("@semantic_primitive").At("name").AsString().append(inputData.At("name").AsString());
//                refBody = grs::Body(bodyname);
//            }
//            if(i->first == "coordinateFrame")
//            {
//                std::string name = i->second.At("@semantic_primitive").At("name").AsString().append(inputData.At("name").AsString());
//                coordFrame = grs::OrientationFrame(name);
//            }
//        }
//         
//        grs::PositionCoordinatesSemantics positionCoordSemantics(tarPoint,tarBody,refPoint,refBody,coordFrame);
//        #ifdef VERBOSE_CHECK_KDLE
//            cout << "    position =" << positionCoordSemantics << endl;
//        #endif
//
//        Variant const coordinates = inputData.At("coordinates");
//        for (Variant::ConstMapIterator i=coordinates.MapBegin(); i != coordinates.MapEnd(); ++i)
//        {
//        
//        }
//    }
//    
//    if(geomtype == "Orientation")
//    {
//        Variant const semantics = inputData.At("semantics");
//        for (Variant::ConstMapIterator i=semantics.MapBegin(); i != semantics.MapEnd(); ++i)
//        {
//            if(i->first == "target")
//            {
//                std::string framename = i->second.At("frame").At("@semantic_primitive").At("name").AsString().append(inputData.At("name").AsString());
//                tarFrame = grs::OrientationFrame(framename);
//                std::string bodyname = i->second.At("body").At("@semantic_primitive").At("name").AsString().append(inputData.At("name").AsString()); 
//                tarBody = grs::Body(bodyname);
//                
//            }
//            if(i->first == "reference")
//            {
//                std::string framename = i->second.At("frame").At("@semantic_primitive").At("name").AsString().append(inputData.At("name").AsString());
//                refFrame = grs::OrientationFrame(framename);
//                std::string bodyname = i->second.At("body").At("@semantic_primitive").At("name").AsString().append(inputData.At("name").AsString());
//                refBody = grs::Body(bodyname);
//            }
//            if(i->first == "coordinateFrame")
//            {
//                std::string name = i->second.At("@semantic_primitive").At("name").AsString() + inputData.At("name").AsString().append(inputData.At("name").AsString());
//                coordFrame = grs::OrientationFrame(name);
//            }
//        }
//         
//        grs::OrientationCoordinatesSemantics orientationCoordSemantics(tarFrame,tarBody,refFrame,refBody,coordFrame);
//        #ifdef VERBOSE_CHECK_KDLE
//            cout << "orientation =" << orientationCoordSemantics << endl;
//        #endif
//        Variant const coordinates = inputData.At("coordinates");
//        for (Variant::ConstMapIterator i=coordinates.MapBegin(); i != coordinates.MapEnd(); ++i)
//        {
//        
//        }
//    }
//    
//    if(geomtype == "Pose")
//    {
//        Variant const semantics = inputData.At("semantics");
//        for (Variant::ConstMapIterator i=semantics.MapBegin(); i != semantics.MapEnd(); ++i)
//        {
//            if(i->first == "target")
//            {
//                std::string pointname = i->second.At("point").At("@semantic_primitive").At("name").AsString().append(inputData.At("name").AsString());
//                tarPoint = grs::Point(pointname);
//                std::string framename = i->second.At("frame").At("@semantic_primitive").At("name").AsString().append(inputData.At("name").AsString()); 
//                tarFrame = grs::OrientationFrame(framename);
//                std::string bodyname = i->second.At("body").At("@semantic_primitive").At("name").AsString().append(inputData.At("name").AsString());
//                tarBody = grs::Body(bodyname);
//                
//            }
//            if(i->first == "reference")
//            {
//                std::string pointname = i->second.At("point").At("@semantic_primitive").At("name").AsString().append(inputData.At("name").AsString());
//                refPoint = grs::Point(pointname);
//                std::string framename = i->second.At("frame").At("@semantic_primitive").At("name").AsString().append(inputData.At("name").AsString());
//                refFrame = grs::OrientationFrame(framename);
//                std::string bodyname = i->second.At("body").At("@semantic_primitive").At("name").AsString().append(inputData.At("name").AsString());
//                refBody = grs::Body(bodyname);
//            }
//            if(i->first == "coordinateFrame")
//            {
//                std::string name = i->second.At("@semantic_primitive").At("name").AsString().append(inputData.At("name").AsString());
//                coordFrame = grs::OrientationFrame(name);
//            }
//        }
//        
//        Variant const coordinates = inputData.At("coordinates");
//        
//        for (Variant::ConstMapIterator i=coordinates.MapBegin(); i != coordinates.MapEnd(); ++i)
//        {
//            if(i->first == "rows")
//            {
//                Eigen::Matrix<double, 4, 4 > Matrix4d;
//                
//                unsigned int matrixrow(0);
//                for (Variant::ConstListIterator k = i->second.ListBegin(); k != i->second.ListEnd(); ++k)
//                {
//                    unsigned int matrixcolumn(0);
//                    for (Variant::ConstListIterator j = k->ListBegin(); j != k->ListEnd(); ++j)
//                    {
//                        
//                        Matrix4d(matrixrow,matrixcolumn) = j->AsDouble();
//                        matrixcolumn++;
//                    }
//                    matrixrow++;
//                }
//                
//                for(unsigned int i=0; i<3; i++)
//                {
//                    T.p.data[i] = Matrix4d(i,3);
//                }
//                
//                unsigned int k(0);
//                for(unsigned int i=0; i<3; i++)
//                {
//                    for(unsigned int j=0; j<3; j++)
//                    {
//                        T.M.data[k] = Matrix4d(i,j);
//                        k++;
//                    }
//                    
//                }
//            }
//        }
//    }
//   
//
//    return grs::Pose<KDL::Frame>(grs::PoseCoordinatesSemantics(tarPoint, tarFrame,tarBody, refPoint, refFrame,refBody,coordFrame),T);
//}


//this function should be made a template which returns a kdle kinematics (Segment, Link)
//based on the value of @kinematicstype

//kdle::Segment<grs::Pose<KDL::Frame> >const& jsonToKinematics(Variant const& inputData)
//{
//
//    grs::Pose<KDL::Frame> rootFrame, rootFrame2, tipFrame, jointFrame;
//    grs::Pose<KDL::Vector, KDL::Rotation> rootFrame3;
//    grs::Position<KDL::Vector> rootFrame4;
//    kdle::Link< grs::Pose<KDL::Frame> > link;
//    std::vector< kdle::AttachmentFrame<grs::Pose<KDL::Frame> > > frameList1;
//    
//    std::string kinematicstype = inputData.At("@kinematicstype").AsString();
//    if(kinematicstype == "Segment")
//    { 
//        std::cout << kinematicstype << std::endl;
//        
//        Variant const jointsemantics = inputData.At("link").At("@kinematics");
//        for (Variant::ConstMapIterator i=jointsemantics.MapBegin(); i != jointsemantics.MapEnd(); ++i)
//        {
//            if(i->first == "rootFrame")
//            {
//                jsonToPose(i->second.At("@geometry"), rootFrame);
//                std::cout << "rootFrame= " << rootFrame << std::endl;
//                
//            }
//            if(i->first == "tipFrame")
//            {
//                jsonToPose(i->second.At("@geometry"), tipFrame);
//                std::cout << "tipFrame= " << tipFrame << std::endl;
//            }
//        }
//        
//        link = kdle::Link< grs::Pose<KDL::Frame> > (inputData.At("link").At("@kinematics").At("name").AsString(), rootFrame, tipFrame);
//        
//        Variant const attachmentframes = inputData.At("framelist");
//        for (Variant::ConstListIterator k=attachmentframes.ListBegin(); k != attachmentframes.ListEnd(); ++k)
//        {
//            jsonToPose (k->At("frame").At("@geometry"), jointFrame);
//            std::cout <<"Joint pose= " << jointFrame << std::endl;
//            frameList1.push_back( kdle::createAttachmentFrame(jointFrame, kdle::FrameType::JOINT));
//        }
//    }
//    return kdle::Segment<grs::Pose<KDL::Frame> >(inputData.At("name").AsString(),link,frameList1);
//    
//    if(kinematicstype == "Joint")
//    {
//        std::cout << kinematicstype << std::endl;        
//        Variant const jointsemantics = inputData.At("@jointtype");
//        for (Variant::ConstMapIterator i=jointsemantics.MapBegin(); i != jointsemantics.MapEnd(); ++i)
//        {
//            if(i->first == "@constraint")
//            {
//                
//            }
//            if(i->first == "target")
//            {
//                std::string jointname= i->second.At("frameid").At("name").AsString();
//                std::string jointid= i->second.At("frameid").At("id").AsString();
//                std::cout << "targetframename= " << jointname << std::endl;
//                
//            }
//            if(i->first == "reference")
//            {
//                std::string jointname= i->second.At("frameid").At("name").AsString();
//                std::string jointid= i->second.At("frameid").At("id").AsString();
//                std::cout << "referenceframename= " << jointname << std::endl;
//            }
//        }
//    
//    }
//    
//    if(kinematicstype == "KinematicChain")
//    {
//        Variant const jointlist = inputData.At("jointlist");
//        for (Variant::ConstListIterator i=jointlist.ListBegin(); i != jointlist.ListEnd(); ++i)
//        {
//            if(i->At("isRoot").AsBool() == true)
//            {
//                std::string jointname = i->At("@jointid").At("name").AsString();
//            }
//            else
//            {
//            
//            }
//        }
//    
//    }
//}
#endif	/* JSON_TO_SEMANTICS_PARSER_HPP */

