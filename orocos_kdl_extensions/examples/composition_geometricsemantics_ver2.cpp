/* 
 * File:   compositiontest.cpp
 * Author: azamat
 *
 * Created on December 21, 2011, 11:46 AM
 */


#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/frames_io.hpp>
//#include <kdl_extensions/functionalcomputation_kdl.hpp>
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
//using namespace kdle;

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
        grs::PoseCoordinatesSemantics pose_joint1_B_semantics("j1","J1","Segment1.Joint1","b","B","Base","B");
        KDL::Vector joint1_position1_B = KDL::Vector(0.1,0,0); //position of joint frame's origin
        grs::Position<KDL::Vector> joint1_origin_position("j1","Segment1","b", "Base", "B", joint1_position1_B);

        KDL::Rotation joint1_coord_orientation1_B = Rotation::RotZ(0.0);
        grs::Orientation<KDL::Rotation> joint1_origin_orientation("J1","Segment1", "B","Base","B", joint1_coord_orientation1_B);
        grs::Pose<KDL::Vector, KDL::Rotation> joint1_pose_B(joint1_origin_position, joint1_origin_orientation);

        KDL::Vector joint1_rotation_axis;
        double starting_angle = joint1_coord_orientation1_B.GetRotAngle(joint1_rotation_axis,0.00001); //rotation axis
        Joint joint1 = Joint("Segment1.Joint1", joint1_position1_B, joint1_rotation_axis, Joint::RotAxis, 1, 0, 0.01);

        //Link1 tip frame1 wrt B
        KDL::Vector link1_tip_position1_B = Vector(0.4, 0.0, 0.0);
        grs::Position<KDL::Vector> link1_tip_position("l1","Segment1","b", "Base", "B", link1_tip_position1_B);

        KDL::Rotation link1_tip_coord_orientation1_B = Rotation::Identity();
        grs::Orientation<KDL::Rotation> link1_tip_orientation("L1","Segment1", "B","Base","B", link1_tip_coord_orientation1_B);
        grs::Pose<KDL::Vector, KDL::Rotation> link1_tip_pose_B(link1_tip_position, link1_tip_orientation);

        grs::Orientation<KDL::Rotation> temp_orientation_l1_b;
        KDL::Rotation tempRot;
        if (link1_tip_pose_B.getOrientation(temp_orientation_l1_b))
            tempRot = temp_orientation_l1_b.getCoordinates().getCoordinates();

        grs::Position<KDL::Vector> temp_position_l1_b;
        KDL::Vector tempVector;
        if(link1_tip_pose_B.getPosition(temp_position_l1_b))
            tempVector = temp_position_l1_b.getCoordinates().getCoordinates();
        KDL::Frame link1_tip_pose_B_coord(tempRot, tempVector );
        Segment segment1 = Segment("Segment1.Link1", joint1, link1_tip_pose_B_coord);
    //~SEGMENT METADATA
        
    //POSE
        //Link1 tip with respect to joint1 at 0 defines the length of the segment
        grs::Pose<KDL::Vector, KDL::Rotation> pose_l1_j1_0 = grs::compose(joint1_pose_B.inverse2(),link1_tip_pose_B);

        // joint1 with respect to Base/World at some value q=M_PI/2.0  
        KDL::Rotation joint1_coord_orientation1_q_B = joint1.pose(q(0)).M;
        grs::Orientation<KDL::Rotation> joint1_origin_orientation_q_B("J1","Segment1", "B","Base","B", joint1_coord_orientation1_q_B);
        grs::Pose<KDL::Vector, KDL::Rotation> joint1_pose_q_B(joint1_origin_position, joint1_origin_orientation_q_B);
        //Link tip with respect to B while q is changing (segment.pose(q))
        grs::Pose<KDL::Vector,KDL::Rotation> pose_l1_b_q = grs::compose(joint1_pose_q_B, pose_l1_j1_0);
//        if(pose_l1_b_q.getCoordinates().getCoordinates() == segment1.pose(q(0)) )
        { 
            std::cout <<"pose_l1_j1_0  " << pose_l1_j1_0 << std::endl << std::endl;   
            std::cout <<"joint1_pose_q_B  " << joint1_pose_q_B << std::endl << std::endl;   
            std::cout <<"Tip L1 with respect to B at value q  " << pose_l1_b_q << std::endl << std::endl<< segment1.pose(q(0))<<std::endl;    
            cout << endl;    
        }
    //~POSES
    
    //TWISTS
    std::cout << std::endl<< std::endl<< "TWISTS " <<  std::endl<<std::endl<< std::endl;
    //j1 on Segment1.Joint1 twist w.r.t Base
    grs::LinearVelocityCoordinatesSemantics linear_vel_coord_seman_j1("j1","Segment1","Base","B");
    KDL::Vector coordinatesLinearVelocity_j1 = joint1.twist(qdot(0)).vel;
    grs::LinearVelocity<KDL::Vector> linearVelocity_j1(linear_vel_coord_seman_j1 ,coordinatesLinearVelocity_j1);

    grs::AngularVelocityCoordinatesSemantics ang_vel_coord_seman_j1("Segment1","Base","B");
    KDL::Vector coordinatesAngularVelocity_j1 = joint1.twist(qdot(0)).rot;
    grs::AngularVelocity<KDL::Vector> angularVelocity_j1(ang_vel_coord_seman_j1, coordinatesAngularVelocity_j1);
    //joint.twist returns this.
    grs::Twist<KDL::Vector,KDL::Vector> twist_j1(linearVelocity_j1, angularVelocity_j1);
    
    // l1 on Segment1.Link1 twist w.r.t Base
    //distance between points j1 and l1 at changing value of q; at 0 it is pose_l1_j1_0.p
    // needs to be put in different coordinates with the same origin.
    grs::Position<KDL::Vector> temp_position_l1_j1_q;
    grs::Orientation<KDL::Rotation> temp_orientation_l1_j1_q;
    if(pose_l1_j1_0.getPosition(temp_position_l1_j1_q))
    {
        temp_position_l1_j1_q = temp_position_l1_j1_q.inverse();
        std::cout << "pose_l1_j1_0 position inv " << temp_position_l1_j1_q << std::endl;
    
        if (joint1_pose_q_B.getOrientation(temp_orientation_l1_j1_q))
        {
            std::cout <<"joint1_pose_q_B orientation " << temp_orientation_l1_j1_q << std::endl;
        }
    }   
    if(temp_position_l1_j1_q.changeCoordinateFrame(temp_orientation_l1_j1_q))
//    grs::PositionCoordinates<KDL::Vector> distance = joint1_pose_q_B.getCoordinates().getCoordinates().M * ;
//    grs::Position<KDL::Vector> position_l1_j1_q("j1","Segment1","l1", "Segment1", "J1", distance);
    std::cout<< "distance " << temp_position_l1_j1_q << std::endl;
    
    grs::Orientation<KDL::Rotation> temp_orientation;
    if(pose_l1_b_q.getOrientation(temp_orientation))
    {
        temp_orientation=temp_orientation.inverse();
//    grs::Orientation<KDL::Rotation> orientation_l1_j1_q("J1","Segment1", "L1","Segment1","L1", orientation);
        std::cout<< "orientation " << temp_orientation << std::endl;
    }
    if(twist_j1.changePointBody(temp_position_l1_j1_q))
    {
        std::cout << "Change reference point of joint twist " << std::endl << twist_j1 << std::endl;//segment.twist returns this. Segment tip twist with respect to previous segment tip
        //M.Inv(segment.twist) returns this. Segment tip twist with respect to joint frame
        if(twist_j1.changeCoordinateFrame(temp_orientation))
        {
            std::cout << "J1 TWIST CONTRIBUTION " << std::endl << twist_j1 << std::endl;
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
    twist_j1_unit.changePointBody(temp_position_l1_j1_q);
    twist_j1_unit.changeCoordinateFrame(temp_orientation);
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
    if(acctwist_j1.changePointBody(temp_position_l1_j1_q))
    {
        if(acctwist_j1.changeCoordinateFrame(temp_orientation))
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
    
  
    grs::Position<KDL::Vector> position_l1_b_q("b","Base","l1", "Segment1", "B", temp_position_l1_j1_q.getCoordinates());
    grs::Orientation<KDL::Rotation> orientation_l1_b_q("B","Base", "L1","Segment1","L1", temp_orientation.getCoordinates());
    
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
    
    return 0;
}