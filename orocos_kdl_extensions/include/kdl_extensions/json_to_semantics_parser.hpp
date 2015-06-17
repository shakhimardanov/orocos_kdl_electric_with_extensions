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

typedef struct
{
    std::string point;
    std::string frame;
    std::string body;
} SemanticData;

using libvariant::Variant;
//this function should be made a template which returns a kdle geometry (Pose, Twist)
//based on the value of @geomtype
grs::Pose<KDL::Frame> jsonToGeometry(Variant const& inputData, std::vector<SemanticData>& semanticData);

//this function should be made a template which returns a kdle kinematics (Segment, Link)
//based on the value of @kinematicstype
void jsonToKinematics(Variant const& inputData, std::vector<SemanticData>& semanticData,kdle::Segment<grs::Pose<KDL::Frame> >& segment);

void walkJSONTree(Variant const& inputData, std::vector<SemanticData>& semanticData);

void walkJSONTree(Variant const& inputData,  Agnode_t* node, Agedge_t* edgeVector, Agraph_t *g, int& nodecounter);

void drawTree(Variant const& inputData);

bool createTree(std::string const& inputModelFile, std::string const& inputSchemaFile, std::vector<SemanticData>& semanticData, bool const& plotOff);
#endif	/* JSON_TO_SEMANTICS_PARSER_HPP */

