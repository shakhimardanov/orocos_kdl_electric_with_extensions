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

/* This example is a first attempt to construct a kinematic chain that has
 complete geometric semantics. The segment or joint primitives used in the 
 example are standard KDL primitives and do not take geometric semantics as
 parameters. Therefore, the semantics constraints are validated on poses and twists
 whose specific coordinates (e.g. PoseCoordinates using KDL::Frame) are then used
 to instantiate segment and joint models. Hence, many expressions  in this example 
 * still use such things as matrix multiplication. For an example that uses kinematic
 primitives with geometric semantics check the example chain_composition_geometricsemantics.cpp*/

#define COMPARISON_TEST

#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_extensions/functionalcomputation_kdl.hpp>
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

int main(int argc, char** argv)
{
    KDL::JntArray q(3);
    q(0)=-M_PI/6.0;
    q(1)=M_PI/24.0;
    q(2)=-M_PI/12.0;
    KDL::JntArray qdot(3);
    qdot(0)=0.5;
    qdot(1)=-0.25;
    qdot(2)=0.35;
    KDL::JntArray qdotdot(3);
    qdotdot(0)=0.115;
    qdotdot(1)=-0.225;
    qdotdot(2)=0.375;

//SEGMENT1
    
    //SEGMENT METADATA
    // joint1 with respect to Base/World. In ideal case one should have a frame data to construct a joint
    KDL::Vector joint1_position1_B = KDL::Vector(0.1,0,0); //position of joint frame's origin
    KDL::Rotation joint1_coord_orientation1_B = Rotation::RotZ(0.0);
    KDL::Vector joint1_rotation_axis;
    double starting_angle = joint1_coord_orientation1_B.GetRotAngle(joint1_rotation_axis,0.00001); //rotation axis
    grs::PoseCoordinates<KDL::Frame> pose_coord_joint1_B(KDL::Frame(joint1_coord_orientation1_B, joint1_position1_B));
    grs::PoseCoordinatesSemantics pose_joint1_B_semantics("j1","J1","Segment1.Joint1","b","B","Base","B");
    grs::Pose<KDL::Frame> posejoint1_B(pose_joint1_B_semantics, pose_coord_joint1_B);
    Joint joint1 = Joint("Segment1.Joint1", joint1_position1_B, joint1_rotation_axis, Joint::RotAxis, 1, 0, 0.01);
    
    //Link1 tip frame1 wrt B
    KDL::Vector link1tip_position1_B = Vector(0.4, 0.0, 0.0);
    KDL::Rotation link1tip_coord_orientation1_B = Rotation::Identity();
    grs::PoseCoordinates<KDL::Frame> pose_coord_link1tip_B(KDL::Frame(link1tip_coord_orientation1_B, link1tip_position1_B));
    grs::PoseCoordinatesSemantics pose_link1tip_B_semantics("l1","L1","Segment1.Link1","b","B","Base","B");
    grs::Pose<KDL::Frame> poselink1tip_B(pose_link1tip_B_semantics, pose_coord_link1tip_B);
    KDL::Frame tip_frame1 = poselink1tip_B.getCoordinates().getCoordinates();
    Segment segment1 = Segment("Segment1.Link1", joint1, tip_frame1);
    //~SEGMENT METADATA
    
    //POSES
    std::cout << std::endl<< std::endl<< "POSES" <<  std::endl<<std::endl<< std::endl;
    //Link1 tip with respect to joint1 at 0 defines the length of the segment
    grs::Pose<KDL::Frame> pose_l1_j1_0 = grs::compose(posejoint1_B.inverse2(), poselink1tip_B);
    // joint1 with respect to Base/World at some value q=M_PI/2.0    
    KDL::Rotation joint1_coord_orientation1_q_B = joint1.pose(q(0)).M;
    grs::PoseCoordinates<KDL::Frame> pose_coord_joint1_q_B(KDL::Frame(joint1_coord_orientation1_q_B, joint1_position1_B));
    grs::Pose<KDL::Frame> posejoint1_q_B(pose_joint1_B_semantics, pose_coord_joint1_q_B);
    //Link tip with respect to B while q is changing (segment.pose(q))
    grs::Pose<KDL::Frame> pose_l1_b_q = grs::compose(posejoint1_q_B, pose_l1_j1_0);
    if(pose_l1_b_q.getCoordinates().getCoordinates() == segment1.pose(q(0)) )
    {
        std::cout <<"Tip L1 with respect to B at value q  " << pose_l1_b_q << std::endl;    
        cout << endl;    
    }
    //~POSES
    
    //TWISTS
    std::cout << std::endl<< std::endl<< "TWISTS " <<  std::endl<<std::endl<< std::endl;
    //j1 on Segment1.Joint1 twist w.r.t Base
    grs::LinearVelocityCoordinatesSemantics linear_vel_coord_seman_j1("j1","Segment1","Base","J1");
    KDL::Vector coordinatesLinearVelocity_j1 = joint1.twist(qdot(0)).vel;
    grs::LinearVelocity<KDL::Vector> linearVelocity_j1(linear_vel_coord_seman_j1 ,coordinatesLinearVelocity_j1);

    grs::AngularVelocityCoordinatesSemantics ang_vel_coord_seman_j1("Segment1","Base","J1");
    KDL::Vector coordinatesAngularVelocity_j1 = joint1.twist(qdot(0)).rot;
    grs::AngularVelocity<KDL::Vector> angularVelocity_j1(ang_vel_coord_seman_j1, coordinatesAngularVelocity_j1);
    //joint.twist returns this.
    grs::Twist<KDL::Vector,KDL::Vector> twist_j1(linearVelocity_j1, angularVelocity_j1);
    
    // l1 on Segment1.Link1 twist w.r.t Base
    //distance between points j1 and l1 at changing value of q; at 0 it is pose_l1_j1_0.p
    // needs to be put in different coordinates with the same origin.
    grs::PositionCoordinates<KDL::Vector> distance = posejoint1_q_B.getCoordinates().getCoordinates().M * (-1*pose_l1_j1_0.getCoordinates().getCoordinates().p);
    grs::Position<KDL::Vector> position_l1_j1_q("j1","Segment1","l1", "Segment1", "J1", distance);
    //std::cout<< "distance " << distance << std::endl;
    
    grs::OrientationCoordinates<KDL::Rotation> orientation = pose_l1_b_q.getCoordinates().getCoordinates().M.Inverse();
    grs::Orientation<KDL::Rotation> orientation_l1_j1_q("J1","Segment1", "L1","Segment1","L1", orientation);
    //std::cout<< "orientation " << orientation << std::endl;
    
    if(twist_j1.changePointBody(position_l1_j1_q))
    {
        std::cout << "Change reference point of joint twist " << std::endl << twist_j1 << std::endl;//segment.twist returns this. Segment tip twist with respect to previous segment tip
        if(twist_j1.changeCoordinateFrame(orientation_l1_j1_q))
        {
            std::cout << "J1 TWIST CONTRIBUTION " << std::endl << twist_j1 << std::endl; //M.Inv(segment.twist) returns this. Segment tip twist with respect to joint frame
        }
    }
    //l1 on Segment1.Link1 twist w.r.t Joint1 expressed in L1
    //for the 1st link twist01 = twist_j1 (in terms of values)
    grs::LinearVelocityCoordinatesSemantics twist01_lin_sem("l1","Segment1","Segment1", "L1");
    KDL::Vector twist01_lin_coord = twist_j1.getLinearVelocity().getCoordinates().getCoordinates();
    grs::LinearVelocity<KDL::Vector> twist01_lin(twist01_lin_sem,twist01_lin_coord);
    
    grs::AngularVelocityCoordinatesSemantics twist01_ang_sem("Segment1","Segment1", "L1");
    KDL::Vector twist01_ang_coord = twist_j1.getAngularVelocity().getCoordinates().getCoordinates();
    grs::AngularVelocity<KDL::Vector> twist01_ang(twist01_ang_sem, twist01_ang_coord);
    
  
    grs::Twist<KDL::Vector, KDL::Vector> twist01(twist01_lin, twist01_ang);
    std::cout << "L1 TWIST01 " << std::endl << twist01 << std::endl << std::endl;
  
    //UNIT TWIST in Segment tip frame
    grs::LinearVelocityCoordinatesSemantics linear_vel_coord_seman_j1_unit("j1","Segment1","Base","J1");
    KDL::Vector coordinatesLinearVelocity_j1_unit = joint1.twist(1.0).vel;
    grs::LinearVelocity<KDL::Vector> linearVelocity_j1_unit(linear_vel_coord_seman_j1_unit ,coordinatesLinearVelocity_j1_unit);

    grs::AngularVelocityCoordinatesSemantics ang_vel_coord_seman_j1_unit("Segment1","Base","J1");
    KDL::Vector coordinatesAngularVelocity_j1_unit = joint1.twist(1.0).rot;
    grs::AngularVelocity<KDL::Vector> angularVelocity_j1_unit(ang_vel_coord_seman_j1_unit, coordinatesAngularVelocity_j1_unit);
    
    //unit joint.twist
    grs::Twist<KDL::Vector,KDL::Vector> twist_j1_unit(linearVelocity_j1_unit, angularVelocity_j1_unit);
    
    //unit twist in segment tip frame
    //it is computed in the same manner as the tip frame twist
    twist_j1_unit.changePointBody(position_l1_j1_q);
    twist_j1_unit.changeCoordinateFrame(orientation_l1_j1_q);
    std::cout << "L1 UNIT TWIST " << std::endl << twist_j1_unit << std::endl;
    
    //ACC TWIST
    std::cout << std::endl<< std::endl<< std::endl<< "ACCTWISTS " <<  std::endl<<std::endl<< std::endl;
    //acctwist consists of 3 components: PARENT acctwist; JOINT acctwist; BIAS acctwist
    //all the constraints and operation on velocity twists apply here
    
    //JOINT ACCTWIST
    //it can also be computed by multiplying qdotdot with unit twist
    //here we compute by changing its ref. point
    grs::LinearVelocityCoordinatesSemantics linear_acc_coord_seman_j1("j1","Segment1","Base","J1");
    KDL::Vector coordinatesLinearAcc_j1 = joint1.twist(qdotdot(0)).vel;
    grs::LinearVelocity<KDL::Vector> linearAcc_j1(linear_acc_coord_seman_j1 ,coordinatesLinearAcc_j1);

    grs::AngularVelocityCoordinatesSemantics ang_acc_coord_seman_j1("Segment1","Base","J1");
    KDL::Vector coordinatesAngularAcc_j1 = joint1.twist(qdotdot(0)).rot;
    grs::AngularVelocity<KDL::Vector> angularAcc_j1(ang_acc_coord_seman_j1, coordinatesAngularAcc_j1);

    grs::Twist<KDL::Vector,KDL::Vector> acctwist_j1(linearAcc_j1, angularAcc_j1);    
    if(acctwist_j1.changePointBody(position_l1_j1_q))
    {
        if(acctwist_j1.changeCoordinateFrame(orientation_l1_j1_q))
        {
            std::cout << "J1 ACCTWIST CONTRIBUTION " << std::endl << acctwist_j1 << std::endl; //M.Inv(segment.twist) returns this. Segment tip twist with respect to joint frame
        }
    }
   
    //BIAS ACCTWIST
    grs::LinearVelocityCoordinatesSemantics biasacctwist01_lin_sem("l1","Segment1","Segment1", "L1");
    KDL::Vector biasacctwist01_lin_coord = twist01.getAngularVelocity().getCoordinates().getCoordinates()*twist_j1.getLinearVelocity().getCoordinates().getCoordinates() + twist01.getLinearVelocity().getCoordinates().getCoordinates()*twist_j1.getAngularVelocity().getCoordinates().getCoordinates();
    grs::LinearVelocity<KDL::Vector> biasacctwist01_lin(biasacctwist01_lin_sem, biasacctwist01_lin_coord);
    
    grs::AngularVelocityCoordinatesSemantics biasacctwist01_ang_sem("Segment1","Segment1", "L1");
    KDL::Vector biasacctwist01_ang_coord = twist01.getAngularVelocity().getCoordinates().getCoordinates()*twist_j1.getAngularVelocity().getCoordinates().getCoordinates();
    grs::AngularVelocity<KDL::Vector> biasacctwist01_ang(biasacctwist01_ang_sem, biasacctwist01_ang_coord);
    
    grs::Twist<KDL::Vector, KDL::Vector> biasacctwist01(biasacctwist01_lin, biasacctwist01_ang);
    std::cout << "BIAS ACCTWIST CONTRIBUTION " << std::endl << biasacctwist01 << std::endl<< std::endl;;
    
    //PARENT(here it is BASE) ACCTWIST
    grs::LinearVelocityCoordinatesSemantics acctwist00_lin_sem("b","Base","World", "B");
    KDL::Vector acctwist00_lin_coord = KDL::Vector(0, 0, 9.8);
    grs::LinearVelocity<KDL::Vector> acctwist00_lin(acctwist00_lin_sem, acctwist00_lin_coord);
    
    grs::AngularVelocityCoordinatesSemantics acctwist00_ang_sem("Base","World", "B");
    KDL::Vector acctwist00_ang_coord = KDL::Vector::Zero();
    grs::AngularVelocity<KDL::Vector> acctwist00_ang(acctwist00_ang_sem, acctwist00_ang_coord);
    grs::Twist<KDL::Vector, KDL::Vector> acctwist00(acctwist00_lin, acctwist00_ang);
    
  
    grs::Position<KDL::Vector> position_l1_b_q("b","Base","l1", "Segment1", "B", distance);
    grs::Orientation<KDL::Rotation> orientation_l1_b_q("B","Base", "L1","Segment1","L1", orientation);
    
    if(acctwist00.changePointBody(position_l1_b_q))
    { 
        if(acctwist00.changeCoordinateFrame(orientation_l1_b_q))
        {
            std::cout << "L1 ACCTWIST PARENT " << std::endl << acctwist00 << std::endl;
        }
    }
    std::cout << std::endl << "L1 ACCTWIST COMPOSITIONS" << std::endl<< std::endl;
    grs::Twist<KDL::Vector, KDL::Vector> acctwist1 = grs::compose(grs::compose(acctwist_j1,acctwist00), biasacctwist01);
    std::cout << "L1 ACCTWIST " << std::endl << acctwist1 << std::endl;
    //~TWISTS
//~SEGMENT1

//SEGMENT2
    //SEGMENT METADATA    
    // joint2 with respect to L1
    KDL::Vector joint2_position2_L1 = KDL::Vector(0.0,0,0); //position of joint frame's origin
    KDL::Rotation joint2_coord_orientation2_L1 = Rotation::RotZ(0.0);
    KDL::Vector joint2_rotation_axis;
    starting_angle = joint2_coord_orientation2_L1.GetRotAngle(joint2_rotation_axis,0.00001); //rotation axis    
    grs::PoseCoordinates<KDL::Frame> pose_coord_joint2_L1(KDL::Frame(joint2_coord_orientation2_L1, joint2_position2_L1));
    grs::PoseCoordinatesSemantics pose_joint2_L1_semantics("j2","J2","Segment2.Joint2","l1","L1","Segment1.Link1","L1");
    grs::Pose<KDL::Frame> posejoint2_L1(pose_joint2_L1_semantics, pose_coord_joint2_L1);
    Joint joint2 = Joint("Segment2.Joint2", joint2_position2_L1, joint2_rotation_axis, Joint::RotAxis, 1, 0, 0.01);
    
     
    //Link2 tip frame2 w.r.t L1
    KDL::Vector link2tip_position2_L1 = Vector(0.4, 0.0, 0.0);
    KDL::Rotation link2tip_coord_orientation2_L1 = Rotation::Identity();
    grs::PoseCoordinates<KDL::Frame> pose_coord_link2tip_L1(KDL::Frame(link2tip_coord_orientation2_L1, link2tip_position2_L1));
    grs::PoseCoordinatesSemantics pose_link2tip_L1_semantics("l2","L2","Segment2.Link2","l1","L1","Segment1.Link1","L1");
    grs::Pose<KDL::Frame> poselink2tip_L1(pose_link2tip_L1_semantics, pose_coord_link2tip_L1);
    KDL::Frame tip_frame2 = poselink2tip_L1.getCoordinates().getCoordinates();
    Segment segment2 = Segment("Segment2.Link2", joint2, tip_frame2);
    //~SEGMENT METADATA
    
    //POSES
    std::cout << std::endl<< std::endl<< "POSES" <<  std::endl<<std::endl<< std::endl;
    //Link2 tip with respect to joint2 at 0 defines the length of the segment
    grs::Pose<KDL::Frame> pose_l2_j2_0 = grs::compose(posejoint2_L1.inverse2() ,poselink2tip_L1);
    // joint2 with respect to L1 at some value q=M_PI/2.0    
    Rotation joint2_coord_orientation2_q_L1 = joint2.pose(q(1)).M;
    grs::PoseCoordinates<KDL::Frame> pose_coord_joint2_q_L1(KDL::Frame(joint2_coord_orientation2_q_L1, joint2_position2_L1));
    grs::Pose<KDL::Frame> posejoint2_q_L1(pose_joint2_L1_semantics, pose_coord_joint2_q_L1);
    //Link tip with respect to L1 while q is changing
    grs::Pose<KDL::Frame> pose_l2_l1_q = grs::compose(posejoint2_q_L1, pose_l2_j2_0);
    if(pose_l2_l1_q.getCoordinates().getCoordinates() == segment2.pose(q(1)) )
    {
        std::cout <<"Tip L2 with respect to L1 at value q  " << pose_l2_l1_q << std::endl;    
        cout << endl;    
    }
    //~POSES
    
    //TWISTS
    std::cout << std::endl<< std::endl<< std::endl<< std::endl<< "TWISTS " <<  std::endl << std::endl;
   //j2 on Segment2.Joint2 twist w.r.t Segment1.Link1
    grs::LinearVelocityCoordinatesSemantics linear_vel_coord_seman_j2("j2","Segment2","Segment1","J2");
    KDL::Vector coordinatesLinearVelocity_j2 = joint2.twist(qdot(1)).vel;
    grs::LinearVelocity<KDL::Vector> linearVelocity_j2(linear_vel_coord_seman_j2 ,coordinatesLinearVelocity_j2);

    grs::AngularVelocityCoordinatesSemantics ang_vel_coord_seman_j2("Segment2","Segment1","J2");
    KDL::Vector coordinatesAngularVelocity_j2 = joint2.twist(qdot(1)).rot;
    grs::AngularVelocity<KDL::Vector> angularVelocity_j2(ang_vel_coord_seman_j2, coordinatesAngularVelocity_j2);
    
    //joint.twist returns this.
    grs::Twist<KDL::Vector,KDL::Vector> twist_j2(linearVelocity_j2, angularVelocity_j2);
    
    // l2 on Segment2.Link2 twist w.r.t Segment1.Link1
    //distance between points j2 and l2 at changing value of q; at 0 it is pose_l2_j2_0.p
    // needs to be put in different coordinates with the same origin.
    grs::PositionCoordinates<KDL::Vector> distance2 = posejoint2_q_L1.getCoordinates().getCoordinates().M * (-1*pose_l2_j2_0.getCoordinates().getCoordinates().p);
    grs::Position<KDL::Vector> position_l2_j2_q("j2","Segment2","l2", "Segment2", "J2", distance2);
    
    grs::OrientationCoordinates<KDL::Rotation> orientation2 = pose_l2_l1_q.getCoordinates().getCoordinates().M.Inverse(); 
    grs::Orientation<KDL::Rotation> orientation_l2_j2_q("J2","Segment2", "L2","Segment2","L2", orientation2);
    
    if(twist_j2.changePointBody(position_l2_j2_q))
    {
        if(twist_j2.changeCoordinateFrame(orientation_l2_j2_q))
        {
            std::cout << "J2 TWIST CONTRIBUTION " << std::endl << twist_j2 << std::endl; //M.Inv(segment.twist) returns this. Segment tip twist with respect to joint frame
        }
    }
    //l1 on Segment1.Link1 twist w.r.t Joint1 expressed in L1
    //for the 2st link twist02 = twist01 + twist_j2 (in terms of values)
    grs::Position<KDL::Vector> position_l2_l1_q("l1","Segment1","l2", "Segment2", "L1", distance2);
    twist01.changePointBody(position_l2_l1_q);
    // std::cout << std::endl<< std::endl<< std::endl<<"CHANGE FRAME OF LINK " << std::endl<< std::endl<< std::endl;
    grs::Orientation<KDL::Rotation> orientation_l2_l1_q("L1","Segment1", "L2","Segment2","L2", orientation2);
    twist01.changeCoordinateFrame(orientation_l2_l1_q);
    std::cout << std::endl<< std::endl<< std::endl<<"LINK CONTRIBUTION " << std::endl << twist01 << std::endl<< std::endl;
    std::cout << std::endl<< std::endl<< std::endl<<"COMPOSITION " << std::endl<< std::endl<< std::endl;
    grs::Twist<KDL::Vector, KDL::Vector> twist02 = grs::compose(twist_j2,twist01);
    std::cout << "L2 TWIST02 " << std::endl << twist02 << std::endl << std::endl;
    
    //UNIT TWIST in Segment tip frame
    grs::LinearVelocityCoordinatesSemantics linear_vel_coord_seman_j2_unit("j2","Segment2","Segment1","J2");
    KDL::Vector coordinatesLinearVelocity_j2_unit = joint2.twist(1.0).vel;
    grs::LinearVelocity<KDL::Vector> linearVelocity_j2_unit(linear_vel_coord_seman_j2_unit ,coordinatesLinearVelocity_j2_unit);

    grs::AngularVelocityCoordinatesSemantics ang_vel_coord_seman_j2_unit("Segment2","Segment1","J2");
    KDL::Vector coordinatesAngularVelocity_j2_unit = joint2.twist(1.0).rot;
    grs::AngularVelocity<KDL::Vector> angularVelocity_j2_unit(ang_vel_coord_seman_j2_unit, coordinatesAngularVelocity_j2_unit);
    
    //unit joint.twist
    grs::Twist<KDL::Vector,KDL::Vector> twist_j2_unit(linearVelocity_j2_unit, angularVelocity_j2_unit);
    
    //unit twist in segment tip frame
    //it is computed in the same manner as the tip frame twist
    twist_j2_unit.changePointBody(position_l2_j2_q);
    twist_j2_unit.changeCoordinateFrame(orientation_l2_j2_q);
    std::cout << "L2 UNIT TWIST " << std::endl << twist_j2_unit << std::endl;
    
    //ACC TWIST
    std::cout << std::endl<< std::endl<< std::endl<< "ACCTWISTS " <<  std::endl<<std::endl<< std::endl;
    //acctwist consists of 3 components: PARENT acctwist; JOINT acctwist; BIAS acctwist
    //all the constraints and operation on velocity twists apply here
    
    //JOINT ACCTWIST
    //it can also be computed by multiplying qdotdot with unit twist
    //here we compute by changing its ref. point
    grs::LinearVelocityCoordinatesSemantics linear_acc_coord_seman_j2("j2","Segment2","Segment1","J2");
    KDL::Vector coordinatesLinearAcc_j2 = joint2.twist(qdotdot(1)).vel;
    grs::LinearVelocity<KDL::Vector> linearAcc_j2(linear_acc_coord_seman_j2 ,coordinatesLinearAcc_j2);

    grs::AngularVelocityCoordinatesSemantics ang_acc_coord_seman_j2("Segment2","Segment1","J2");
    KDL::Vector coordinatesAngularAcc_j2 = joint2.twist(qdotdot(1)).rot;
    grs::AngularVelocity<KDL::Vector> angularAcc_j2(ang_acc_coord_seman_j2, coordinatesAngularAcc_j2);

    grs::Twist<KDL::Vector,KDL::Vector> acctwist_j2(linearAcc_j2, angularAcc_j2);    
    if(acctwist_j2.changePointBody(position_l2_j2_q))
    {
        if(acctwist_j2.changeCoordinateFrame(orientation_l2_j2_q))
        {
            std::cout << "J2 ACCTWIST CONTRIBUTION " << std::endl << acctwist_j2 << std::endl; //M.Inv(segment.twist) returns this. Segment tip twist with respect to joint frame
        }
    }
   
    //BIAS ACCTWIST
    grs::LinearVelocityCoordinatesSemantics biasacctwist02_lin_sem("l2","Segment2","Segment2", "L2");
    KDL::Vector biasacctwist02_lin_coord = twist02.getAngularVelocity().getCoordinates().getCoordinates()*twist_j2.getLinearVelocity().getCoordinates().getCoordinates() + twist02.getLinearVelocity().getCoordinates().getCoordinates()*twist_j2.getAngularVelocity().getCoordinates().getCoordinates();
    grs::LinearVelocity<KDL::Vector> biasacctwist02_lin(biasacctwist02_lin_sem, biasacctwist02_lin_coord);
    
    grs::AngularVelocityCoordinatesSemantics biasacctwist02_ang_sem("Segment2","Segment2", "L2");
    KDL::Vector biasacctwist02_ang_coord = twist02.getAngularVelocity().getCoordinates().getCoordinates()*twist_j2.getAngularVelocity().getCoordinates().getCoordinates();
    grs::AngularVelocity<KDL::Vector> biasacctwist02_ang(biasacctwist02_ang_sem, biasacctwist02_ang_coord);
    
    grs::Twist<KDL::Vector, KDL::Vector> biasacctwist02(biasacctwist02_lin, biasacctwist02_ang);
    std::cout << "BIAS ACCTWIST CONTRIBUTION " << std::endl << biasacctwist02 << std::endl<< std::endl;;
    
    //PARENT(here it is BASE) ACCTWIST
    // std::cout << std::endl<< std::endl<< std::endl<<"CHANGE FRAME OF LINK " << std::endl<< std::endl<< std::endl;
    if(acctwist1.changePointBody(position_l2_l1_q))
    { 
        if(acctwist1.changeCoordinateFrame(orientation_l2_l1_q))
        {
            std::cout << "L2 ACCTWIST PARENT " << std::endl << acctwist1 << std::endl;
        }
    }
    std::cout << std::endl << "L2 ACCTWIST COMPOSITIONS" << std::endl<< std::endl;
    grs::Twist<KDL::Vector, KDL::Vector> acctwist2 = grs::compose(grs::compose(acctwist_j2,acctwist1), biasacctwist02);
    std::cout << "L2 ACCTWIST " << std::endl << acctwist2 << std::endl;
    //~TWISTS
//~SEGMENT2

//SEGMENT3
    //SEGMENT METADATA
    // joint3 with respect to L2
    Joint joint3 = Joint("Segment3.Joint3", Vector(-0.1, 0.0, 0),Vector(0,0,1),Joint::RotAxis, 1, 0, 0.01);
    grs::PoseCoordinatesSemantics pose_joint3_L2("j3","J3","Segment3.Joint3","l2","L2","Segment2.Link2","L2");
    Vector joint3_position3_L2 = joint3.JointOrigin();
    Rotation joint3_coord_orientation3_L2=joint3.pose(0).M;
    grs::PoseCoordinates<KDL::Frame> pose_coord_joint3_L2(KDL::Frame(joint3_coord_orientation3_L2, joint3_position3_L2));
    grs::Pose<KDL::Frame> posejoint3_L2(pose_joint3_L2, pose_coord_joint3_L2);

    //Link3 tip frame3
    grs::PoseCoordinatesSemantics pose_link3tip_L2("l3","L3","Segment3.Link3","l2","L2","Segment2.Link2","L2");
    Vector link3tip_position3_L2 = Vector(0.4, 0.0, 0);
    Rotation link3tip_coord_orientation3_L2 = Rotation::Identity();
    grs::PoseCoordinates<KDL::Frame> pose_coord_link3tip_L2(KDL::Frame(link3tip_coord_orientation3_L2, link3tip_position3_L2));
    grs::Pose<KDL::Frame> poselink3tip_L2(pose_link3tip_L2, pose_coord_link3tip_L2);
    Frame frame3 = poselink3tip_L2.getCoordinates().getCoordinates();
    Segment segment3 = Segment("Link3", joint3, frame3);
    //~SEGMENT METADATA
    
    //POSES
    std::cout << std::endl<< std::endl<< "POSES" <<  std::endl<<std::endl<< std::endl;
    //Link3 tip with respect to joint3 at 0 defines the length of the segment
    grs::Pose<KDL::Frame> pose_l3_j3_0 = grs::compose(posejoint3_L2.inverse2() ,poselink3tip_L2);
    // joint3 with respect to L2 at some value q=M_PI/2.0    
    Rotation joint3_coord_orientation3_q_L2=joint3.pose(q(2)).M;
    grs::PoseCoordinates<KDL::Frame> pose_coord_joint3_q_L2(KDL::Frame(joint3_coord_orientation3_q_L2, joint3_position3_L2));
    grs::Pose<KDL::Frame> posejoint3_q_L2(pose_joint3_L2, pose_coord_joint3_q_L2);
    //Link tip with respect to L2 while q is changing
    grs::Pose<KDL::Frame> pose_l3_l2_q = grs::compose(posejoint3_q_L2, pose_l3_j3_0);
    if( pose_l3_l2_q.getCoordinates().getCoordinates() == segment3.pose(q(2)) )
    {
        std::cout <<"Tip L3 with respect to L2 at value q  " << pose_l3_l2_q << std::endl;    
        cout << endl;    
    }
    
    //~POSES
    
    //TWISTS
    std::cout << std::endl<< std::endl<< std::endl<< std::endl<< "TWISTS " <<  std::endl << std::endl;
    //j3 on Segment3.Joint3 twist w.r.t Segment2.Link2
    grs::LinearVelocityCoordinatesSemantics linear_vel_coord_seman_j3("j3","Segment3","Segment2","J3");
    KDL::Vector coordinatesLinearVelocity_j3 = joint3.twist(qdot(2)).vel;
    grs::LinearVelocity<KDL::Vector> linearVelocity_j3(linear_vel_coord_seman_j3 ,coordinatesLinearVelocity_j3);

    grs::AngularVelocityCoordinatesSemantics ang_vel_coord_seman_j3("Segment3","Segment2","J3");
    KDL::Vector coordinatesAngularVelocity_j3 = joint3.twist(qdot(2)).rot;
    grs::AngularVelocity<KDL::Vector> angularVelocity_j3(ang_vel_coord_seman_j3, coordinatesAngularVelocity_j3);
    
    //joint.twist returns this.
    grs::Twist<KDL::Vector,KDL::Vector> twist_j3(linearVelocity_j3, angularVelocity_j3);
    
    // l1 on Segment1.Link1 twist w.r.t Base
    //distance between points j1 and l1 at changing value of q; at 0 it is pose_l1_j1_0.p
    // needs to be put in different coordinates with the same origin.
    grs::PositionCoordinates<KDL::Vector> distance3 = posejoint3_q_L2.getCoordinates().getCoordinates().M * (-1* pose_l3_j3_0.getCoordinates().getCoordinates().p);
    grs::Position<KDL::Vector> position_l3_j3_q("j3","Segment3","l3", "Segment3", "J3", distance3);
    //std::cout<< "distance " << distance3 << std::endl;
    
    grs::OrientationCoordinates<KDL::Rotation> orientation3 = pose_l3_l2_q.getCoordinates().getCoordinates().M.Inverse(); 
    grs::Orientation<KDL::Rotation> orientation_l3_j3_q("J3","Segment3", "L3","Segment3","L3", orientation3);
    //std::cout<< "orientation " << orientation3 << std::endl;
    
    if(twist_j3.changePointBody(position_l3_j3_q))
    {
         if(twist_j3.changeCoordinateFrame(orientation_l3_j3_q))
         {
            std::cout << "J3 TWIST CONTRIBUTION " << std::endl << twist_j3 << std::endl; //M.Inv(segment.twist) returns this. Segment tip twist with respect to joint frame
         }
    }
    
    //l1 on Segment1.Link1 twist w.r.t Joint1 expressed in L1
    //for the 2st link twist02 = twist01 + twist_j2 (in terms of values)
    grs::Position<KDL::Vector> position_l3_l2_q("l2","Segment2","l3", "Segment3", "L2", distance3);
    twist02.changePointBody(position_l3_l2_q);
    grs::Orientation<KDL::Rotation> orientation_l3_l2_q("L2","Segment2", "L3","Segment3","L3", orientation3);
    //std::cout << "Change reference point of L2 twist02 " << std::endl << twist02 << std::endl;
    twist02.changeCoordinateFrame(orientation_l3_l2_q);
    std::cout << std::endl<< std::endl<< std::endl<<"LINK CONTRIBUTION " << std::endl << twist02 << std::endl<< std::endl;
    std::cout << std::endl<< std::endl<< std::endl<<"COMPOSITION " << std::endl<< std::endl<< std::endl;
    grs::Twist<KDL::Vector, KDL::Vector> twist03 = grs::compose(twist02,twist_j3);
    std::cout << "L3 TWIST03 " << twist03 << std::endl << std::endl;
    
    //UNIT TWIST in Segment tip frame
    grs::LinearVelocityCoordinatesSemantics linear_vel_coord_seman_j3_unit("j3","Segment3","Segment2","J3");
    KDL::Vector coordinatesLinearVelocity_j3_unit = joint3.twist(1.0).vel;
    grs::LinearVelocity<KDL::Vector> linearVelocity_j3_unit(linear_vel_coord_seman_j3_unit ,coordinatesLinearVelocity_j3_unit);

    grs::AngularVelocityCoordinatesSemantics ang_vel_coord_seman_j3_unit("Segment3","Segment2","J3");
    KDL::Vector coordinatesAngularVelocity_j3_unit = joint3.twist(1.0).rot;
    grs::AngularVelocity<KDL::Vector> angularVelocity_j3_unit(ang_vel_coord_seman_j3_unit, coordinatesAngularVelocity_j3_unit);
    
    //unit joint.twist
    grs::Twist<KDL::Vector,KDL::Vector> twist_j3_unit(linearVelocity_j3_unit, angularVelocity_j3_unit);
    
    //unit twist in segment tip frame
    //it is computed in the same manner as the tip frame twist
    twist_j3_unit.changePointBody(position_l3_j3_q);
    twist_j3_unit.changeCoordinateFrame(orientation_l3_j3_q);
    std::cout << "L3 UNIT TWIST " << std::endl << twist_j3_unit << std::endl;

    //ACC TWIST
    std::cout << std::endl<< std::endl<< std::endl<< "ACCTWISTS " <<  std::endl<<std::endl<< std::endl;
    //acctwist consists of 3 components: PARENT acctwist; JOINT acctwist; BIAS acctwist
    //all the constraints and operation on velocity twists apply here
    
    //JOINT ACCTWIST
    //it can also be computed by multiplying qdotdot with unit twist
    //here we compute by changing its ref. point
    grs::LinearVelocityCoordinatesSemantics linear_acc_coord_seman_j3("j3","Segment3","Segment2","J3");
    KDL::Vector coordinatesLinearAcc_j3 = joint3.twist(qdotdot(2)).vel;
    grs::LinearVelocity<KDL::Vector> linearAcc_j3(linear_acc_coord_seman_j3 ,coordinatesLinearAcc_j3);

    grs::AngularVelocityCoordinatesSemantics ang_acc_coord_seman_j3("Segment3","Segment2","J3");
    KDL::Vector coordinatesAngularAcc_j3 = joint3.twist(qdotdot(2)).rot;
    grs::AngularVelocity<KDL::Vector> angularAcc_j3(ang_acc_coord_seman_j3, coordinatesAngularAcc_j3);

    grs::Twist<KDL::Vector,KDL::Vector> acctwist_j3(linearAcc_j3, angularAcc_j3);    
    if(acctwist_j3.changePointBody(position_l3_j3_q))
    {
        if(acctwist_j3.changeCoordinateFrame(orientation_l3_j3_q))
        {
            std::cout << "J3 ACCTWIST CONTRIBUTION " << std::endl << acctwist_j3 << std::endl; //M.Inv(segment.twist) returns this. Segment tip twist with respect to joint frame
        }
    }
   
    //BIAS ACCTWIST
    grs::LinearVelocityCoordinatesSemantics biasacctwist03_lin_sem("l3","Segment3","Segment3", "L3");
    KDL::Vector biasacctwist03_lin_coord = twist03.getAngularVelocity().getCoordinates().getCoordinates()*twist_j3.getLinearVelocity().getCoordinates().getCoordinates() + twist03.getLinearVelocity().getCoordinates().getCoordinates()*twist_j3.getAngularVelocity().getCoordinates().getCoordinates();
    grs::LinearVelocity<KDL::Vector> biasacctwist03_lin(biasacctwist03_lin_sem, biasacctwist03_lin_coord);
    
    grs::AngularVelocityCoordinatesSemantics biasacctwist03_ang_sem("Segment3","Segment3", "L3");
    KDL::Vector biasacctwist03_ang_coord = twist03.getAngularVelocity().getCoordinates().getCoordinates()*twist_j3.getAngularVelocity().getCoordinates().getCoordinates();
    grs::AngularVelocity<KDL::Vector> biasacctwist03_ang(biasacctwist03_ang_sem, biasacctwist03_ang_coord);
    
    grs::Twist<KDL::Vector, KDL::Vector> biasacctwist03(biasacctwist03_lin, biasacctwist03_ang);
    std::cout << "BIAS ACCTWIST CONTRIBUTION " << std::endl << biasacctwist03 << std::endl<< std::endl;;
    
    //PARENT(here it is BASE) ACCTWIST
    // std::cout << std::endl<< std::endl<< std::endl<<"CHANGE FRAME OF LINK " << std::endl<< std::endl<< std::endl;
    if(acctwist2.changePointBody(position_l3_l2_q))
    { 
        if(acctwist2.changeCoordinateFrame(orientation_l3_l2_q))
        {
            std::cout << "L3 ACCTWIST PARENT " << std::endl << acctwist2 << std::endl;
        }
    }
    std::cout << std::endl << "L3 ACCTWIST COMPOSITIONS" << std::endl<< std::endl;
    grs::Twist<KDL::Vector, KDL::Vector> acctwist3 = grs::compose(grs::compose(acctwist_j3,acctwist2), biasacctwist03);
    std::cout << "L3 ACCTWIST " << std::endl << acctwist3 << std::endl;
    
    //~TWISTS
    
//~SEGMENT3

    //FK
    grs::Pose<KDL::Frame> fk_pose_L3_B_q = grs::compose(pose_l1_b_q, grs::compose(pose_l2_l1_q,pose_l3_l2_q) );
    std::cout <<"Tip L3 with respect to B at value q  " << fk_pose_L3_B_q << std::endl;    
    cout << endl;  

    #ifdef COMPARISON_TEST   
        KDL::Chain achain;
        achain.addSegment(segment1);
        achain.addSegment(segment2);
        achain.addSegment(segment3);



        ChainFkSolverPos_recursive fksolvertest(achain);
        ChainFkSolverVel_recursive fkvelsolvertest(achain);
        ChainIdSolver_RNE idsolvertest(achain, KDL::Vector(0,0,-9.8));
        Frame tempEEtest[3];  
        Frame tempLocal[3];  
        Twist tempTwist[3];
        FrameVel tempVeltest[3];  
        KDL::JntArray jointInitialPose(achain.getNrOfJoints());
        jointInitialPose(0)= q(0);
        jointInitialPose(1)= q(1);
        jointInitialPose(2)= q(2);

        KDL::JntArrayVel jointInitialRate(achain.getNrOfJoints());
        jointInitialRate.q(0)= q(0);
        jointInitialRate.q(1)= q(1);
        jointInitialRate.q(2)= q(2);

        jointInitialRate.qdot(0)= qdot(0);
        jointInitialRate.qdot(1)= qdot(1);
        jointInitialRate.qdot(2)= qdot(2);
        KDL::JntArray torques(achain.getNrOfJoints());
        KDL::Wrenches fext;
        fext.resize(achain.getNrOfSegments(),KDL::Wrench());

        std::cout << idsolvertest.CartToJnt(q,qdot,qdotdot, fext, torques) << std::endl;
        std::cout << std::endl << std::endl<< std::endl << std::endl;

        for(unsigned int i=0; i<achain.getNrOfSegments();i++)
        {   
            std::cout <<"Tip Frame " << achain.getSegment(i).getFrameToTip()<< std::endl;
            tempLocal[i]=achain.getSegment(i).pose(jointInitialPose(i));
            std::cout <<"Segment tip pose " << achain.getSegment(i).pose(jointInitialPose(i)) << std::endl;
            std::cout <<"Joint twist " << achain.getSegment(i).getJoint().twist(jointInitialRate.qdot(i)) << std::endl;
            std::cout <<"Segment tip twist " << achain.getSegment(i).twist(jointInitialPose(i), jointInitialRate.qdot(i)) << std::endl;
            std::cout <<"Segment tip twist inv " << tempLocal[i].M.Inverse(achain.getSegment(i).twist(jointInitialPose(i), jointInitialRate.qdot(i))) << std::endl;
            std::cout <<"Segment tip Unit twist inv " << tempLocal[i].M.Inverse(achain.getSegment(i).twist(jointInitialPose(i), 1.0)) << std::endl;
            std::cout <<"Segment tip ACC twist Joint Cont" << tempLocal[i].M.Inverse(achain.getSegment(i).twist(jointInitialPose(i), 1.0))*qdotdot(i) << std::endl;
            if(i == 0)
            {
               tempTwist[i] = tempLocal[i].M.Inverse(achain.getSegment(i).twist(jointInitialPose(i), jointInitialRate.qdot(i)));
               std::cout <<"Total twist " << tempTwist[i] << std::endl;
            }
            if(i!=0)
            {       
               tempTwist[i] = tempLocal[i].Inverse(tempTwist[i-1]) + tempLocal[i].M.Inverse(achain.getSegment(i).twist(jointInitialPose(i), jointInitialRate.qdot(i))) ;
               std::cout <<"Total twist " << tempTwist[i] << std::endl;
            }

        }
    #endif
    
    return 0;
}