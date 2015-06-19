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

/* This is a compact versoin of the composition_geometricsemantics_ver2.cpp example.
 * It abstracts the semantic info for the complete kinematic chain into a class 
 * of its own. This is then used with a set of functions to recursively instantiate
 * the segments and the joints of the kinematic chain. This example is condensed, 
 * thus a little difficult to follow, since it assumes that one is already knowledgeable
 * with the API of geometric relations library.
 */


#include <graphviz/gvc.h>
#include <graphviz/graph.h>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <Position/Position.h>
#include <Orientation/Orientation.h>
#include <Pose/Pose.h>
#include <LinearVelocity/LinearVelocity.h>
#include <AngularVelocity/AngularVelocity.h>
#include <Twist/Twist.h>
#include <Force/Force.h>
#include <Torque/Torque.h>
#include <Wrench/Wrench.h>

#include <Position/PositionCoordinatesKDL.h>
#include <Orientation/OrientationCoordinatesKDL.h>
#include <Pose/PoseCoordinatesKDL.h>
#include <LinearVelocity/LinearVelocityCoordinatesKDL.h>
#include <AngularVelocity/AngularVelocityCoordinatesKDL.h>
#include <Twist/TwistCoordinatesKDL.h>
#include <Force/ForceCoordinatesKDL.h>
#include <Torque/TorqueCoordinatesKDL.h>
#include <Wrench/WrenchCoordinatesKDL.h>

namespace grs = geometric_semantics;

using namespace std;
using namespace KDL;


typedef struct
{
    std::string point;
    std::string frame;
    std::string body;
} ChainSemanticData;

typedef struct
{
    KDL::Vector joint_origin_position_coord;
    KDL::Rotation joint_origin_orientation_coord;
    KDL::Vector link_tip_position_coord;
    KDL::Rotation link_tip_orientation_coord;
    double joint_inertia;
    KDL::RigidBodyInertia link_inertia;
} ChainGeometricInertialData;



void initChainSemantics(ChainSemanticData *bodyframes, const unsigned int size)
{
    bodyframes[0].point = "w";
    bodyframes[0].frame = "W";
    bodyframes[0].body = "World";
    
    bodyframes[1].point = "b";
    bodyframes[1].frame = "B";
    bodyframes[1].body = "Base";
    
    bodyframes[2].point = "j1";
    bodyframes[2].frame = "J1";
    bodyframes[2].body = "Segment1";

    bodyframes[3].point = "l1";
    bodyframes[3].frame = "L1";
    bodyframes[3].body = "Segment1";
    
    bodyframes[4].point = "j2";
    bodyframes[4].frame = "J2";
    bodyframes[4].body = "Segment2";
    
    bodyframes[5].point = "l2";
    bodyframes[5].frame = "L2";
    bodyframes[5].body = "Segment2";
    
    bodyframes[6].point = "j3";
    bodyframes[6].frame = "J3";
    bodyframes[6].body = "Segment3";

    bodyframes[7].point = "l3";
    bodyframes[7].frame = "L3";
    bodyframes[7].body = "Segment3";

    bodyframes[8].point = "j4";
    bodyframes[8].frame = "J4";
    bodyframes[8].body = "Segment4";

    bodyframes[9].point = "l4";
    bodyframes[9].frame = "L4";
    bodyframes[9].body = "Segment4";
    
    bodyframes[10].point = "j5";
    bodyframes[10].frame = "J5";
    bodyframes[10].body = "Segment5";

    bodyframes[11].point = "l5";
    bodyframes[11].frame = "L5";
    bodyframes[11].body = "Segment5";    
}

void initChainGeometry(ChainGeometricInertialData *segmentgeoms, const unsigned int size)
{
    segmentgeoms[0].joint_origin_position_coord = KDL::Vector(0,0,0);
    segmentgeoms[0].joint_origin_orientation_coord =  KDL::Rotation::RotZ(0.0);
    segmentgeoms[0].link_tip_position_coord = KDL::Vector(0.2,0.0,0.0);
    segmentgeoms[0].link_tip_orientation_coord = KDL::Rotation::Identity();
    //    RotationalInertia rotInerSeg1(0.0, 0.0, 0.0, 0.0, 0.0, 0.0); //around symmetry axis of rotation
    //    double pointMass = 0.25; //in kg
    //    RigidBodyInertia inerSegment1(pointMass, Vector(0.0, -0.4, 0.0), rotInerSeg1);
    segmentgeoms[1].joint_origin_position_coord = KDL::Vector(0,0,0);
    segmentgeoms[1].joint_origin_orientation_coord =  KDL::Rotation::RotZ(0.0);
    segmentgeoms[1].link_tip_position_coord = KDL::Vector(0.3,0.0,0.0);
    segmentgeoms[1].link_tip_orientation_coord = KDL::Rotation::Identity();

    segmentgeoms[2].joint_origin_position_coord = KDL::Vector(0,0,0);
    segmentgeoms[2].joint_origin_orientation_coord =  KDL::Rotation::RotZ(0.0);
    segmentgeoms[2].link_tip_position_coord = KDL::Vector(0.4,0.0,0.0);
    segmentgeoms[2].link_tip_orientation_coord = KDL::Rotation::Identity();

    segmentgeoms[3].joint_origin_position_coord = KDL::Vector(0,0,0);
    segmentgeoms[3].joint_origin_orientation_coord =  KDL::Rotation::RotZ(0.0);
    segmentgeoms[3].link_tip_position_coord = KDL::Vector(0.5,0.0,0.0);
    segmentgeoms[3].link_tip_orientation_coord = KDL::Rotation::Identity();

    segmentgeoms[4].joint_origin_position_coord = KDL::Vector(0,0,0);
    segmentgeoms[4].joint_origin_orientation_coord =  KDL::Rotation::RotZ(0.0);
    segmentgeoms[4].link_tip_position_coord = KDL::Vector(0.6,0.0,0.0);
    segmentgeoms[4].link_tip_orientation_coord = KDL::Rotation::Identity();
}

int main(int argc, char** argv)
{
    unsigned int numberOfDOF = 5;
    KDL::JntArray q(numberOfDOF);
    q(0)=-M_PI/6.0;
    q(1)=M_PI/24.0;
    q(2)=-M_PI/12.0;
    q(3)=-M_PI/24.0;
    q(4)=M_PI/18.0;
    
    KDL::JntArray qdot(numberOfDOF);
    qdot(0)=0.5;
    qdot(1)=-0.25;
    qdot(2)=0.35;
    qdot(3)=-0.15;
    qdot(4)=-0.45;
    KDL::JntArray qdotdot(numberOfDOF);
    qdotdot(0)=0.115;
    qdotdot(1)=-0.225;
    qdotdot(2)=0.375;
    qdotdot(3)=-0.425;
    qdotdot(4)=0.1375;

    unsigned int size = numberOfDOF*2+2; //each segment has 2 x body,point,frame + world and base
    
    //init chain semantic data
    ChainSemanticData bodyframes[size];
    initChainSemantics(bodyframes, size);
    
    // init chagin geometric data
    ChainGeometricInertialData segmentgeoms[numberOfDOF];
    initChainGeometry(segmentgeoms, numberOfDOF);

    //containment for chain joint/segment and their names
    std::vector<KDL::Joint> chainjoints;
    std::vector<std::string> chainjointnames;
    std::vector<KDL::Segment> chainlinks;
    std::vector<std::string> chainlinknames;
    
    //create vectors of joint and link names
    //based on the semantic data
    for(unsigned int i=2;i<size; i=i+2)
    {
        chainjointnames.push_back(bodyframes[i].body);
        chainlinknames.push_back(bodyframes[i+1].body);
    }
    
    //keeps geometric data of joint origin at creation time
    std::vector< grs::Pose<KDL::Frame> > joint_poses;
    //keeps geometric data of link tip at creation time
    std::vector< grs::Pose<KDL::Frame> > link_tip_poses;
    //keeps geometric data of link tip with respect to parent tip at value q
    std::vector< grs::Pose<KDL::Frame> > link_tip_poses_relative;

    //keeps geometric data of joint origin twist of value qdot with respect to parent tip and expressed in child link tip frame
    std::vector< grs::Twist<KDL::Vector,KDL::Vector> > joint_twists;
    //keeps geometric data of joint origin twist of value 1.0 (unit twist) with respect to parent tip and expressed in child link tip frame
    std::vector< grs::Twist<KDL::Vector,KDL::Vector> > joint_unit_twists;
    //keeps geometric data of  full link twist composed of joint and parent link contributions expressed in child link tip frame
    std::vector< grs::Twist<KDL::Vector,KDL::Vector> > link_twists;

    //keeps geometric data of joint origin acctwist of value qdotdot with respect to parent tip and expressed in child link tip frame
    std::vector< grs::Twist<KDL::Vector,KDL::Vector> > joint_acctwists;
    //keeps geometric data of  link biasacctwist expressed in child link tip frame
    std::vector< grs::Twist<KDL::Vector,KDL::Vector> > link_biasacctwists;
    //keeps geometric data of  full link acctwist composed of joint, bias and parent link contributions expressed in child link tip frame
    std::vector< grs::Twist<KDL::Vector,KDL::Vector> > link_acctwists;

    unsigned int j=0; // index of the element in the chain
    //instantiate and fill in each vector above
    //this is equivalent to an outward sweep in ID
    for(unsigned int i=2; i<size; i=i+2)
    {
        //SEGMENT DATA
            //joint geometry data; it can be vector of its own
            KDL::Vector joint_rotation_axis;
            double starting_angle = segmentgeoms[j].joint_origin_orientation_coord.GetRotAngle(joint_rotation_axis,0.00001);

            //joint semantic data
            grs::PoseCoordinatesSemantics joint_pose_semantics(bodyframes[i].point,bodyframes[i].frame,bodyframes[i].body,bodyframes[i-1].point,bodyframes[i-1].frame,bodyframes[i-1].body,bodyframes[i-1].frame);
            grs::PoseCoordinates<KDL::Frame> joint_pose_coord(KDL::Frame(segmentgeoms[j].joint_origin_orientation_coord, segmentgeoms[j].joint_origin_position_coord));
            grs::Pose<KDL::Frame> joint_pose(joint_pose_semantics, joint_pose_coord);
            std::cout << "Joint data" << joint_pose <<std::endl;
            KDL::Joint joint = KDL::Joint(chainjointnames[j], segmentgeoms[j].joint_origin_position_coord, joint_rotation_axis, Joint::RotAxis, 1, 0, 0.01);
            joint_poses.push_back(joint_pose);
            chainjoints.push_back(joint);

            // link geometry data; it can be vector of its own
            //link semantic data
            grs::PoseCoordinates<KDL::Frame> link_tip_pose_coord(KDL::Frame(segmentgeoms[j].link_tip_orientation_coord, segmentgeoms[j].link_tip_position_coord));
            grs::PoseCoordinatesSemantics link_tip_pose_semantics(bodyframes[i+1].point,bodyframes[i+1].frame,bodyframes[i+1].body,bodyframes[i-1].point,bodyframes[i-1].frame,bodyframes[i-1].body,bodyframes[i-1].frame);
            grs::Pose<KDL::Frame> link_tip_pose(link_tip_pose_semantics, link_tip_pose_coord);
            std::cout << "Link data "<<link_tip_pose <<std::endl;
            KDL::Frame tip_frame = link_tip_pose.getCoordinates().getCoordinates();
            KDL::Segment segment = KDL::Segment(chainlinknames[j], joint, tip_frame);
            link_tip_poses.push_back(link_tip_pose);
            chainlinks.push_back(segment);
        //~SEGMENT DATA

        //POSES
            //Link tip with respect to joint at 0 defines the length of the segment
            grs::Pose<KDL::Frame> pose_l_j = grs::compose(joint_pose.inverse2(), link_tip_pose);
            // joint pose with respect to previous link tip pose at some value q    
            KDL::Rotation joint_coord_orientation_j_l_q = joint.pose(q(j)).M;
            grs::PoseCoordinates<KDL::Frame> joint_pose_coord_j_l_q(KDL::Frame(joint_coord_orientation_j_l_q, segmentgeoms[j].joint_origin_position_coord));
            grs::Pose<KDL::Frame> joint_pose_j_l_q(joint_pose_semantics, joint_pose_coord_j_l_q);
            //Link tip pose with respect to previous link tip pose at value q (segment.pose(q))
            grs::Pose<KDL::Frame> pose_l_q = grs::compose(joint_pose_j_l_q,pose_l_j);
            if(pose_l_q.getCoordinates().getCoordinates() == chainlinks[j].pose(q(j)) )
            {
                std::cout <<"Link " << j << " tip with respect to previous  link at value q  " << pose_l_q << std::endl;    
                cout << endl;    
            }
            link_tip_poses_relative.push_back(pose_l_q);
        //~POSES

        //TWISTS
            //j on Segment.Joint twist w.r.t previous Segment.Link
            grs::LinearVelocityCoordinatesSemantics joint_linear_vel_semantics(bodyframes[i].point, bodyframes[i].body, bodyframes[i-1].body, bodyframes[i].frame);
            KDL::Vector joint_linear_vel_coordinates = chainjoints[j].twist(qdot(j)).vel;
            grs::LinearVelocity<KDL::Vector> joint_linear_vel(joint_linear_vel_semantics, joint_linear_vel_coordinates);

            grs::AngularVelocityCoordinatesSemantics joint_angular_vel_semantics(bodyframes[i].body, bodyframes[i-1].body, bodyframes[i].frame);
            KDL::Vector joint_angular_vel_coordinates = chainjoints[j].twist(qdot(j)).rot;
            grs::AngularVelocity<KDL::Vector> joint_angular_vel(joint_angular_vel_semantics, joint_angular_vel_coordinates);
            //joint.twist returns this.
            grs::Twist<KDL::Vector,KDL::Vector> joint_twist(joint_linear_vel, joint_angular_vel);
            std::cout << "Joint " << j << " twist relative to previous link tip in joint coordinates " << joint_twist << std::endl;

            // l on Segment.Link twist w.r.t previous link tip frame
            //distance between points j and l at changing value of q; at 0 it is pose_l_j.p
            // needs to be put in different coordinates with the same origin.
            grs::PositionCoordinates<KDL::Vector> distance = joint_pose_j_l_q.getCoordinates().getCoordinates().M * (-1*pose_l_j.getCoordinates().getCoordinates().p);
            grs::Position<KDL::Vector> position_l_j_q(bodyframes[i].point, bodyframes[i].body, bodyframes[i+1].point, bodyframes[i+1].body, bodyframes[i].frame, distance);
            
            grs::OrientationCoordinates<KDL::Rotation> orientation = pose_l_q.getCoordinates().getCoordinates().M.Inverse();
            grs::Orientation<KDL::Rotation> orientation_l_j_q(bodyframes[i].frame, bodyframes[i].body, bodyframes[i+1].frame, bodyframes[i+1].body, bodyframes[i+1].frame, orientation);
            
            //kdl::segment.twist returns this. Segment tip twist with respect to previous segment tip
            if(joint_twist.changePointBody(position_l_j_q))
            {
                if(joint_twist.changeCoordinateFrame(orientation_l_j_q))
                {
                    //M.Inv(segment.twist) returns this. Segment tip twist with respect to joint frame
                    joint_twists.push_back(joint_twist);
                    std::cout << "JOINT TWIST CONTRIBUTION " << std::endl << joint_twists[j] << std::endl; 
                }
            }

            //UNIT TWIST in Segment tip frame (S[i])
            grs::LinearVelocityCoordinatesSemantics joint_unit_twist_linear_vel_coord_semanantics(bodyframes[i].point,bodyframes[i].body,bodyframes[i-1].body, bodyframes[i].frame);
            KDL::Vector joint_unit_twist_linear_vel_coordinates = chainjoints[j].twist(1.0).vel;
            grs::LinearVelocity<KDL::Vector> joint_unit_twist_linear_vel(joint_unit_twist_linear_vel_coord_semanantics,joint_unit_twist_linear_vel_coordinates);

            grs::AngularVelocityCoordinatesSemantics joint_unit_twist_angular_vel_coord_semanantics(bodyframes[i].body, bodyframes[i-1].body, bodyframes[i].frame);
            KDL::Vector joint_unit_twist_angular_vel_coordinates = chainjoints[j].twist(1.0).rot;
            grs::AngularVelocity<KDL::Vector> joint_unit_twist_angular_vel(joint_unit_twist_angular_vel_coord_semanantics, joint_unit_twist_angular_vel_coordinates);
            
            //unit joint.twist
            grs::Twist<KDL::Vector,KDL::Vector> joint_unit_twist(joint_unit_twist_linear_vel, joint_unit_twist_angular_vel);
            
            //unit twist in child segment tip frame
            //it is computed in the same manner as the joint twist contribution above
            if(joint_unit_twist.changePointBody(position_l_j_q))
            {
                if(joint_unit_twist.changeCoordinateFrame(orientation_l_j_q))
                {
                    joint_unit_twists.push_back(joint_unit_twist);
                    std::cout << "UNIT TWIST " << std::endl << joint_unit_twists[j] << std::endl;
                }
            }
            //~UNIT TWIST

            //l on Segment.Link twist w.r.t Segment.Joint expressed in L
            //for the 1st link twist01 = twist_j1 (in terms of values)
            grs::Position<KDL::Vector> position_parent_l_child_l_q(bodyframes[i-1].point, bodyframes[i-1].body, bodyframes[i+1].point, bodyframes[i+1].body, bodyframes[i-1].frame, distance);
            grs::Orientation<KDL::Rotation> orientation_parent_l_child_l_q(bodyframes[i-1].frame,bodyframes[i-1].body, bodyframes[i+1].frame,bodyframes[i+1].body,bodyframes[i+1].frame, orientation);
            if(j != 0)
            {
                //need to copy first and then call methods, otherwise it performs in place modifications
                grs::Twist<KDL::Vector, KDL::Vector> link_twist_copy = link_twists[j-1];
                if(link_twist_copy.changePointBody(position_parent_l_child_l_q))
                {
                    if(link_twist_copy.changeCoordinateFrame(orientation_parent_l_child_l_q))
                    {
                        std::cout << std::endl<< std::endl<< std::endl<<"LINK CONTRIBUTION " << std::endl << link_twist_copy << std::endl<< std::endl;
                        grs::Twist<KDL::Vector, KDL::Vector> link_twist = grs::compose(joint_twists[j],link_twist_copy);
                        link_twists.push_back(link_twist);
                        std::cout << "TOTAL LINK " << j << " TWIST" << std::endl << link_twists[j] << std::endl << std::endl;
                    }
                }     
            }
            else
            {
                grs::LinearVelocityCoordinatesSemantics link_twist_linear_vel_semantics(bodyframes[i+1].point, bodyframes[i+1].body, bodyframes[i].body, bodyframes[i+1].frame);
                KDL::Vector link_twist_linear_vel_coord = joint_twist.getLinearVelocity().getCoordinates().getCoordinates();
                grs::LinearVelocity<KDL::Vector> link_twist_linear_vel(link_twist_linear_vel_semantics, link_twist_linear_vel_coord);
                
                grs::AngularVelocityCoordinatesSemantics link_twist_angular_vel_semantics(bodyframes[i+1].body, bodyframes[i].body, bodyframes[i+1].frame);
                KDL::Vector link_twist_angular_vel_coord = joint_twist.getAngularVelocity().getCoordinates().getCoordinates();
                grs::AngularVelocity<KDL::Vector> link_twist_angular_vel(link_twist_angular_vel_semantics, link_twist_angular_vel_coord);
                
              
                grs::Twist<KDL::Vector, KDL::Vector> link_twist(link_twist_linear_vel, link_twist_angular_vel);
                link_twists.push_back(link_twist);
                std::cout << "TOTAL LINK " << j << " TWIST" << std::endl << link_twists[j] << std::endl << std::endl;
            }
        //~TWISTS

        //ACCTWIST
            //acctwist consists of 3 components: PARENT acctwist; JOINT acctwist; BIAS acctwist
            //all the constraints and operation on velocity twists apply here
            
            //JOINT ACCTWIST
            //it can also be computed by multiplying qdotdot with unit twist
            //here we compute by changing its ref. point
            grs::LinearVelocityCoordinatesSemantics joint_linear_acctwist_coord_semantics(bodyframes[i].point, bodyframes[i].body, bodyframes[i-1].body, bodyframes[i].frame);
            KDL::Vector joint_linear_acctwist_coordinates = chainjoints[j].twist(qdotdot(j)).vel;
            grs::LinearVelocity<KDL::Vector> joint_linear_acctwist(joint_linear_acctwist_coord_semantics, joint_linear_acctwist_coordinates);

            grs::AngularVelocityCoordinatesSemantics joint_ang_acctwist_coord_semantics(bodyframes[i].body, bodyframes[i-1].body, bodyframes[i].frame);
            KDL::Vector joint_ang_acctwist_coordinates = chainjoints[j].twist(qdotdot(j)).rot;
            grs::AngularVelocity<KDL::Vector> joint_ang_acctwist(joint_ang_acctwist_coord_semantics, joint_ang_acctwist_coordinates);

            grs::Twist<KDL::Vector,KDL::Vector> joint_acctwist(joint_linear_acctwist, joint_ang_acctwist);    
            if(joint_acctwist.changePointBody(position_l_j_q))
            {
                if(joint_acctwist.changeCoordinateFrame(orientation_l_j_q))
                { 
                   //M.Inv(segment.twist) returns this. Segment tip twist with respect to joint frame
                    joint_acctwists.push_back(joint_acctwist);
                    std::cout << "JOINT ACCTWIST CONTRIBUTION " << std::endl << joint_acctwists[j] << std::endl;
                }
            }
           
            //BIAS ACCTWIST
            grs::LinearVelocityCoordinatesSemantics link_linear_biasacctwist_coord_semantics(bodyframes[i+1].point, bodyframes[i+1].body, bodyframes[i].body, bodyframes[i+1].frame);
            KDL::Vector link_linear_biasacctwist_coordinates = link_twists[j].getAngularVelocity().getCoordinates().getCoordinates()*joint_twists[j].getLinearVelocity().getCoordinates().getCoordinates() + link_twists[j].getLinearVelocity().getCoordinates().getCoordinates()*joint_twists[j].getAngularVelocity().getCoordinates().getCoordinates();
            grs::LinearVelocity<KDL::Vector> link_linear_biasacctwist(link_linear_biasacctwist_coord_semantics, link_linear_biasacctwist_coordinates);
            
            grs::AngularVelocityCoordinatesSemantics link_ang_biasacctwist_coord_semantics(bodyframes[i+1].body, bodyframes[i].body, bodyframes[i+1].frame);
            KDL::Vector link_ang_biasacctwist_coordinates = link_twists[j].getAngularVelocity().getCoordinates().getCoordinates()*joint_twists[j].getAngularVelocity().getCoordinates().getCoordinates();
            grs::AngularVelocity<KDL::Vector> link_ang_biasacctwist(link_ang_biasacctwist_coord_semantics, link_ang_biasacctwist_coordinates);
            
            grs::Twist<KDL::Vector, KDL::Vector> link_biasacctwist(link_linear_biasacctwist, link_ang_biasacctwist);
            link_biasacctwists.push_back(link_biasacctwist);

            std::cout << "BIAS ACCTWIST CONTRIBUTION " << std::endl << link_biasacctwists[j] << std::endl<< std::endl;;
            
            //PARENT ACCTWIST
            if(j!=0)
            {
                //need to copy first and then call methods, otherwise it performs in place modifications
                grs::Twist<KDL::Vector, KDL::Vector> link_acctwist_copy = link_acctwists[j-1];
                if(link_acctwist_copy.changePointBody(position_parent_l_child_l_q))
                { 
                    if(link_acctwist_copy.changeCoordinateFrame(orientation_parent_l_child_l_q))
                    {
                        std::cout << "LINK ACCTWIST PARENT CONTRIBUTION " << std::endl << link_acctwist_copy << std::endl;
                    }
                }
                grs::Twist<KDL::Vector, KDL::Vector> acctwist = grs::compose(grs::compose(joint_acctwists[j],link_acctwist_copy), link_biasacctwists[j]);
                link_acctwists.push_back(acctwist);
                std::cout << "TOTAL LINK " << j << " ACCTWIST" << std::endl << link_acctwists[j] << std::endl << std::endl;
            }
            else
            {    
                //PARENT(here it is BASE) ACCTWIST
                grs::LinearVelocityCoordinatesSemantics link_linear_acctwist_coord_semantics(bodyframes[i-1].point,bodyframes[i-1].body,bodyframes[i-2].body, bodyframes[i-1].frame);
                KDL::Vector link_linear_acctwist_coordinates = KDL::Vector(0, 0, 9.8);
                grs::LinearVelocity<KDL::Vector> link_linear_acctwist(link_linear_acctwist_coord_semantics, link_linear_acctwist_coordinates);
                
                grs::AngularVelocityCoordinatesSemantics link_ang_acctwist_coord_semantics(bodyframes[i-1].body,bodyframes[i-2].body, bodyframes[i-1].frame);
                KDL::Vector link_ang_acctwist_coordinates = KDL::Vector::Zero();
                grs::AngularVelocity<KDL::Vector> link_ang_acctwist(link_ang_acctwist_coord_semantics, link_ang_acctwist_coordinates);
                grs::Twist<KDL::Vector, KDL::Vector> link_acctwist(link_linear_acctwist, link_ang_acctwist);
                
              
                grs::Position<KDL::Vector> position_l_b_q(bodyframes[i-1].point,bodyframes[i-1].body,bodyframes[i+1].point,bodyframes[i+1].body, bodyframes[i-1].frame, distance);
                grs::Orientation<KDL::Rotation> orientation_l_b_q(bodyframes[i-1].frame,bodyframes[i-1].body, bodyframes[i+1].frame,bodyframes[i+1].body,bodyframes[i+1].frame, orientation);
                
                if(link_acctwist.changePointBody(position_l_b_q))
                { 
                    if(link_acctwist.changeCoordinateFrame(orientation_l_b_q))
                    {
                        std::cout << "LINK ACCTWIST PARENT CONTRIBUTION " << std::endl << link_acctwist << std::endl;
                    }
                }
                
                grs::Twist<KDL::Vector, KDL::Vector> acctwist = grs::compose(grs::compose(joint_acctwists[j],link_acctwist), link_biasacctwists[j]);
                link_acctwists.push_back(acctwist);
                std::cout << "TOTAL LINK " << j << " ACCTWIST" << std::endl << link_acctwists[j] << std::endl << std::endl;
            }
        //~ACCTWIST
        
        // WRENCHES
        //~WRENCHES
        j++;
    }

    for(unsigned int k = 0; k<link_tip_poses_relative.size(); k++)
    {
        std::cout << "Relative link tip poses " << std::endl;
        std::cout << link_tip_poses_relative[k] << std::endl<< std::endl;
    }
    //FK
    grs::Pose<KDL::Frame> fk_pose_L3_B_q = grs::compose(link_tip_poses_relative[0], grs::compose(link_tip_poses_relative[1],link_tip_poses_relative[2]) );
    std::cout <<"Tip L3 with respect to B at value q  " << std::endl<< fk_pose_L3_B_q << std::endl<< std::endl; 

    for(unsigned int i = 0; i<link_twists.size(); i++)
    {
        std::cout << "Relative link tip twists " << std::endl;
        std::cout << link_twists[i] << std::endl<< std::endl;
    }

    for(unsigned int i = 0; i<link_acctwists.size(); i++)
    {
        std::cout << "Relative link tip acctwists " << std::endl;
        std::cout << link_acctwists[i] << std::endl;
    }
    return 0;
}