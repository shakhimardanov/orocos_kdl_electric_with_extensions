#include <kdl_extensions/json_to_semantics_parser.hpp>
#include <iostream>

#include "kdl_extensions/chain_geometric_primitives.hpp"

using namespace libvariant;
using namespace std; 
namespace grs = geometric_semantics;



//this function should be made a template which returns a kdle geometry (Pose, Twist)
//based on the value of @geomtype
grs::PoseCoordinatesSemantics  geometryToKDL(Variant const& inputData, std::vector<SemanticData>& semanticData)
{
//    std::string geomname = inputData.At("name").AsString();
//    std::cout << geomname << std::endl;
//    std::string geomid = inputData.At("id").AsString();
//    std::cout << geomid << std::endl;
//    std::string dsltype = inputData.At("@dsltype").AsString();
//    std::cout << dsltype << std::endl;
    std::string geomtype = inputData.At("@geomtype").AsString();
//    std::cout << geomtype << std::endl;
    
    grs::Point refPoint, tarPoint;
    grs::Body refBody, tarBody;
    grs::OrientationFrame refFrame, tarFrame, coordFrame;
    
    if(geomtype == "Position")
    {
        Variant const semantics = inputData.At("semantics");
        for (Variant::ConstMapIterator i=semantics.MapBegin(); i != semantics.MapEnd(); ++i)
        {
            if(i->first == "target")
            {
                std::string pointname = i->second.Get("point").Get("@semantic_primitive").Get("name").AsString();
                tarPoint = grs::Point(pointname);
                std::string bodyname = i->second.Get("body").Get("@semantic_primitive").Get("name").AsString();
                tarBody = grs::Body(bodyname);
                
            }
            if(i->first == "reference")
            {
                std::string pointname = i->second.Get("point").Get("@semantic_primitive").Get("name").AsString();
                refPoint = grs::Point(pointname);
                std::string bodyname = i->second.Get("body").Get("@semantic_primitive").Get("name").AsString();
                refBody = grs::Body(bodyname);
            }
            if(i->first == "coordinateFrame")
            {
                std::string name = i->second.Get("@semantic_primitive").Get("name").AsString();
                coordFrame = grs::OrientationFrame(name);
            }
        }
         
        grs::PositionCoordinatesSemantics positionCoordSemantics(tarPoint,tarBody,refPoint,refBody,coordFrame);
        cout << "    position =" << positionCoordSemantics << endl;
        
        Variant const coordinates = inputData.At("coordinates");
        for (Variant::ConstMapIterator i=coordinates.MapBegin(); i != coordinates.MapEnd(); ++i)
        {
        
        }
    }
    
    if(geomtype == "Orientation")
    {
        Variant const semantics = inputData.At("semantics");
        for (Variant::ConstMapIterator i=semantics.MapBegin(); i != semantics.MapEnd(); ++i)
        {
            if(i->first == "target")
            {
                std::string framename = i->second.Get("frame").Get("@semantic_primitive").Get("name").AsString();
                tarFrame = grs::OrientationFrame(framename);
                std::string bodyname = i->second.Get("body").Get("@semantic_primitive").Get("name").AsString();
                tarBody = grs::Body(bodyname);
                
            }
            if(i->first == "reference")
            {
                std::string framename = i->second.Get("frame").Get("@semantic_primitive").Get("name").AsString();
                refFrame = grs::OrientationFrame(framename);
                std::string bodyname = i->second.Get("body").Get("@semantic_primitive").Get("name").AsString();
                refBody = grs::Body(bodyname);
            }
            if(i->first == "coordinateFrame")
            {
                std::string name = i->second.Get("@semantic_primitive").Get("name").AsString();
                coordFrame = grs::OrientationFrame(name);
            }
        }
         
        grs::OrientationCoordinatesSemantics orientationCoordSemantics(tarFrame,tarBody,refFrame,refBody,coordFrame);
        cout << "orientation =" << orientationCoordSemantics << endl;
        
        Variant const coordinates = inputData.At("coordinates");
        for (Variant::ConstMapIterator i=coordinates.MapBegin(); i != coordinates.MapEnd(); ++i)
        {
        
        }
    }
    
    if(geomtype == "Pose")
    {
        Variant const semantics = inputData.At("semantics");
        for (Variant::ConstMapIterator i=semantics.MapBegin(); i != semantics.MapEnd(); ++i)
        {
            if(i->first == "target")
            {
                std::string pointname = i->second.Get("point").Get("@semantic_primitive").Get("name").AsString();
                tarPoint = grs::Point(pointname);
                std::string framename = i->second.Get("frame").Get("@semantic_primitive").Get("name").AsString();
                tarFrame = grs::OrientationFrame(framename);
                std::string bodyname = i->second.Get("body").Get("@semantic_primitive").Get("name").AsString();
                tarBody = grs::Body(bodyname);
                
            }
            if(i->first == "reference")
            {
                std::string pointname = i->second.Get("point").Get("@semantic_primitive").Get("name").AsString();
                refPoint = grs::Point(pointname);
                std::string framename = i->second.Get("frame").Get("@semantic_primitive").Get("name").AsString();
                refFrame = grs::OrientationFrame(framename);
                std::string bodyname = i->second.Get("body").Get("@semantic_primitive").Get("name").AsString();
                refBody = grs::Body(bodyname);
            }
            if(i->first == "coordinateFrame")
            {
                std::string name = i->second.Get("@semantic_primitive").Get("name").AsString();
                coordFrame = grs::OrientationFrame(name);
            }
        }
         
//        grs::PoseCoordinatesSemantics poseCoordSemantics(tarPoint, tarFrame,tarBody, refPoint, refFrame,refBody,coordFrame);
//        cout << "pose =" << poseCoordSemantics << endl;
        
        Variant const coordinates = inputData.At("coordinates");
        for (Variant::ConstMapIterator i=coordinates.MapBegin(); i != coordinates.MapEnd(); ++i)
        {
            if(i->first == "rows")
            {
                for (Variant::ConstListIterator k = i->second.ListBegin(); k != i->second.ListEnd(); ++k)
                {
                    for (Variant::ConstListIterator j = k->ListBegin(); j != k->ListEnd(); ++j)
                    {
                        std::cout << "row = " << j->AsDouble() << std::endl;
                    }
                }
            }
        }
    }
   

    return grs::PoseCoordinatesSemantics(tarPoint, tarFrame,tarBody, refPoint, refFrame,refBody,coordFrame);
}


//this function should be made a template which returns a kdle kinematics (Segment, Link)
//based on the value of @kinematicstype
void  kinematicsToKDL(Variant const& inputData, std::vector<SemanticData>& semanticData)
{
//    std::string kinematicsname = inputData.At("name").AsString();
//    std::string kinematicsid = inputData.At("id").AsString();
//    std::string dsltype = inputData.At("@dsltype").AsString();
    std::string kinematicstype = inputData.At("@kinematicstype").AsString();
    std::cout << kinematicstype << std::endl;
    if(kinematicstype == "Segment")
    { 
        grs::PoseCoordinatesSemantics rootFrame, tipFrame, jointFrame;
        Variant const linksemantics = inputData.At("link").Get("@kinematics");
        for (Variant::ConstMapIterator i=linksemantics.MapBegin(); i != linksemantics.MapEnd(); ++i)
        {
            if(i->first == "rootFrame")
            {
                rootFrame = geometryToKDL(i->second.Get("@geometry"), semanticData);
                std::cout << "rootFrame= " << rootFrame << std::endl;
                
            }
            if(i->first == "tipFrame")
            {
                tipFrame = geometryToKDL(i->second.Get("@geometry"), semanticData);
                std::cout << "tipFrame= " << tipFrame << std::endl;
            }
        }
        
        Variant const attachmentframes = inputData.At("framelist");
        for (Variant::ConstListIterator k=attachmentframes.ListBegin(); k != attachmentframes.ListEnd(); ++k)
        {
            jointFrame = geometryToKDL (k->Get("frame").Get("@geometry"), semanticData);
            std::cout << "@frametype= "<< k->Get("@frametype").AsString() << std::endl;
            std::cout << "frame= "<< jointFrame << std::endl;
            
        }
        
    }
    
    return;
}

void walkJSONTree(Variant const& inputData, std::vector<SemanticData>& semanticData)
{
    if (inputData.IsMap())
    {
        #ifdef VERBOSE_WALK
            std::cout << "TOP: IT IS A MAP" << std::endl;
            std::cout << "Size: "<< inputData.Size() << std::endl<<std::endl;
        #endif
        for (Variant::ConstMapIterator i=inputData.MapBegin(); i != inputData.MapEnd(); ++i)
        {
//            if( !(i->second.IsMap() || i->second.IsList()) )
//               switch (i->second.GetType())
//               {
//                case Variant::StringType:
//                    #ifdef VERBOSE_WALK
//                        std::cout << "Key: " << i->first <<std::endl;
//                        std::cout << "Value: "<< inputData.Get(i->first).AsString()<< std::endl<< std::endl;
//                        std::cout << "Value: "<< inputData.At(i->first).AsString()<< std::endl<< std::endl;
//                    #endif  
//                    break;
//                case Variant::FloatType:
//                    #ifdef VERBOSE_WALK
//                        std::cout << "Key: " << i->first <<std::endl;
//                        std::cout << "Value: "<< inputData.Get(i->first).AsDouble()<< std::endl<< std::endl;
//                    #endif  
//                    break;
//                case Variant::BoolType:
//                    #ifdef VERBOSE_WALK
//                        std::cout << "Key: " << i->first <<std::endl;
//                        std::cout << "Value: "<< inputData.Get(i->first).AsBool()<< std::endl<< std::endl;
//                    #endif
//                    break;
//               }
//            else
//            {
//                std::cout << "Key: " << i->first <<std::endl;
                if(i->first == "@kinematics")
                    kinematicsToKDL(i->second, semanticData);
                walkJSONTree(i->second, semanticData);
//            }
        } 
    }
    
    if(inputData.IsList())
    {
        #ifdef VERBOSE_WALK
            std::cout << "TOP: IT IS A LIST" << std::endl;
            printf("Size: %d \n",inputData.Size());
        #endif  
        for (Variant::ConstListIterator j = inputData.ListBegin(); j != inputData.ListEnd(); ++j)
        {
            #ifdef VERBOSE_WALK
                printf("Type of the List element: %d \n", j->GetType());
                std::cout  << std::endl;
            #endif
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
                agsafeset(currentnode, "shape", "oval", "");
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
                agsafeset(currentnode, "shape", "oval", "");

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

#ifdef VERBOSE_WALK
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
