/* 
 * File:   compositiontest.cpp
 * Author: azamat
 *
 * Created on December 21, 2011, 11:46 AM
 */

//#define VERBOSE_CHECK //switches on console output in kdl related methods
 #define VERBOSE_CHECK_MAIN // switches on console output in main

#include <graphviz/gvc.h>
#include <graphviz/graph.h>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_extensions/functionalcomputation_kdltypes.hpp>
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
using namespace kdle;

void createMyTree(KDL::Tree& twoBranchTree);

void drawMyTree(KDL::Tree& twoBranchTree);

void computeTemplatedDynamicsForTree(KDL::Tree& twoBranchTree, KDL::Vector& grav, std::vector<kdle::JointState>& jointState,
                                     std::vector<kdle::SegmentState>& linkState, std::vector<kdle::SegmentState>& linkState2);

void computeRNEDynamicsForChain(KDL::Tree& twoBranchTree, const std::string& rootLink, const std::string& tipLink, KDL::Vector& grav,
                                std::vector<kdle::JointState>& jointState, std::vector<kdle::SegmentState>& linkState);



int main(int argc, char** argv)
{
    KDL::JntArray q(3);
    q(0)=-M_PI/6.0;
    q(1)=M_PI/4.0;
    q(2)=-M_PI/12.0;
    KDL::JntArray qdot(3);
    qdot(0)=0.5;
    qdot(1)=-0.25;
    qdot(2)=0.35;
    
//SEGMENT1

    //SEGMENT METADATA
    // joint1 with respect to Base/World. In ideal case one should have a frame data to construct a joint
    KDL::Vector joint1_position1_B = KDL::Vector(0,0,0); //position of joint frame's origin
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
    //j1 on Segment1.Joint1 twist w.r.t Base
    grs::LinearVelocityCoordinatesSemantics linear_vel_coord_seman_j1("j1","Segment1.Joint1","Base","J1");
    KDL::Vector coordinatesLinearVelocity_j1 = joint1.twist(qdot(0)).vel;
    grs::LinearVelocity<KDL::Vector> linearVelocity_j1(linear_vel_coord_seman_j1 ,coordinatesLinearVelocity_j1);

    grs::AngularVelocityCoordinatesSemantics ang_vel_coord_seman_j1("Segment1.Joint1","Base","J1");
    KDL::Vector coordinatesAngularVelocity_j1 = joint1.twist(qdot(0)).rot;
    grs::AngularVelocity<KDL::Vector> angularVelocity_j1(ang_vel_coord_seman_j1, coordinatesAngularVelocity_j1);
    
    //joint.twist returns this.
    grs::Twist<KDL::Vector,KDL::Vector> twist_j1(linearVelocity_j1, angularVelocity_j1);
    
    // l1 on Segment1.Link1 twist w.r.t Base
    //distance between points j1 and l1 at changing value of q; at 0 it is pose_l1_j1_0.p
    // needs to be put in different coordinates with the same origin.
    grs::PositionCoordinates<KDL::Vector> distance = posejoint1_q_B.getCoordinates().getCoordinates().M * (-1*pose_l1_j1_0.getCoordinates().getCoordinates().p);
    grs::Position<KDL::Vector> position_l1_j1_q("j1","Segment1.Joint1","l1", "Segment1.Link1", "J1", distance);
    std::cout<< "distance " << distance << std::endl;
    
    grs::OrientationCoordinates<KDL::Rotation> orientation = pose_l1_b_q.getCoordinates().getCoordinates().M.Inverse();
    grs::Orientation<KDL::Rotation> orientation_l1_j1_q("J1","Segment1.Joint1", "L1","Segment1.Link1","L1", orientation);
    std::cout<< "orientation " << orientation << std::endl;
    
    bool y = twist_j1.changePointBody(position_l1_j1_q);
    std::cout << "Change reference point of joint twist " << std::endl << twist_j1 << std::endl;//segment.twist returns this. Segment tip twist with respect to previous segment tip
    bool x = twist_j1.changeCoordinateFrame(orientation_l1_j1_q);
    std::cout << "Change coordinate it is expressed in " << std::endl << twist_j1 << std::endl; //M.Inv(segment.twist) returns this. Segment tip twist with respect to joint frame
  
    //l1 on Segment1.Link1 twist w.r.t Joint1 expressed in L1
    //for the 1st link twist01 = twist_j1 (in terms of values)
    grs::LinearVelocityCoordinatesSemantics twist01_lin_sem("l1","Segment1.Link1","Segment1.Joint1", "J1");
    KDL::Vector twist01_lin_coord = twist_j1.getLinearVelocity().getCoordinates().getCoordinates();
    grs::LinearVelocity<KDL::Vector> twist01_lin(twist01_lin_sem,twist01_lin_coord);
    
    grs::AngularVelocityCoordinatesSemantics twist01_ang_sem("Segment1.Link1","Segment1.Joint1", "J1");
    KDL::Vector twist01_ang_coord = twist_j1.getAngularVelocity().getCoordinates().getCoordinates();
    grs::AngularVelocity<KDL::Vector> twist01_ang(twist01_ang_sem, twist01_ang_coord);
    
     //M.Inv(segment.twist) returns this. Segment tip twist with respect to joint frame
    grs::Twist<KDL::Vector, KDL::Vector> twist01(twist01_lin, twist01_ang);
    std::cout << "L1 twist01 " << std::endl << twist01 << std::endl << std::endl;
    
    //~TWISTS
//~SEGMENT1

//SEGMENT2
    //SEGMENT METADATA    
    // joint2 with respect to L1
    KDL::Vector joint2_position2_L1 = KDL::Vector(0,0,0); //position of joint frame's origin
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
    KDL::Frame tip_frame2 = poselink1tip_B.getCoordinates().getCoordinates();
    Segment segment2 = Segment("Segment2.Link2", joint2, tip_frame2);
    //~SEGMENT METADATA
    
    //POSES
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
   //j2 on Segment2.Joint2 twist w.r.t Segment1.Link1
    grs::LinearVelocityCoordinatesSemantics linear_vel_coord_seman_j2("j2","Segment2.Joint2","Segment1.Link1","L1");
    KDL::Vector coordinatesLinearVelocity_j2 = joint2.twist(qdot(1)).vel;
    grs::LinearVelocity<KDL::Vector> linearVelocity_j2(linear_vel_coord_seman_j2 ,coordinatesLinearVelocity_j2);

    grs::AngularVelocityCoordinatesSemantics ang_vel_coord_seman_j2("Segment2.Joint2","Segment1.Link1","L1");
    KDL::Vector coordinatesAngularVelocity_j2 = joint2.twist(qdot(1)).rot;
    grs::AngularVelocity<KDL::Vector> angularVelocity_j2(ang_vel_coord_seman_j2, coordinatesAngularVelocity_j2);
    
    //joint.twist returns this.
    grs::Twist<KDL::Vector,KDL::Vector> twist_j2(linearVelocity_j2, angularVelocity_j2);
    
    // l1 on Segment1.Link1 twist w.r.t Base
    //distance between points j1 and l1 at changing value of q; at 0 it is pose_l1_j1_0.p
    // needs to be put in different coordinates with the same origin.
    grs::PositionCoordinates<KDL::Vector> distance2 = posejoint2_q_L1.getCoordinates().getCoordinates().M * (-1*pose_l2_j2_0.getCoordinates().getCoordinates().p);
    grs::Position<KDL::Vector> position_l2_l1_q("j2","Segment2.Joint2","l2", "Segment1.Link2", "L1", distance2);
    std::cout<< "distance " << distance2 << std::endl;
    
    grs::OrientationCoordinates<KDL::Rotation> orientation2 = pose_l2_l1_q.getCoordinates().getCoordinates().M.Inverse(); 
    grs::Orientation<KDL::Rotation> orientation_l2_j2_q("J1","Segment1.Joint1", "L1","Segment1.Link1","L1", orientation2);
    std::cout<< "orientation " << orientation2 << std::endl;
    
    bool y2 = twist_j2.changePointBody(position_l2_l1_q);
    std::cout << "Change reference point of joint twist " << std::endl << twist_j2 << std::endl;//segment.twist returns this. Segment tip twist with respect to previous segment tip
    bool x2 = twist_j2.changeCoordinateFrame(orientation_l2_j2_q);
    std::cout << "Change coordinate it is expressed in " << std::endl << twist_j2 << std::endl; //M.Inv(segment.twist) returns this. Segment tip twist with respect to joint frame
    
    //l1 on Segment1.Link1 twist w.r.t Joint1 expressed in L1
    //for the 2st link twist02 = twist01 + twist_j2 (in terms of values)
    bool t1 = twist01.changePointBody(position_l2_l1_q);
    if(t1)
        std::cout << "Change reference point of L1 twist01 " << std::endl << twist01 << std::endl;
    
    bool t2 = twist01.changeCoordinateFrame(orientation_l2_j2_q);
    if(t2)
        std::cout << "Change coordinate of L1 twist01 " << std::endl << twist01 << std::endl;
    
//    grs::TwistCoordinatesSemantics twist_coord_sem02("l2","Segment2.Link2","Segment2.Joint2","L2"); 
//    KDL::Twist coordinatesTwist02;
    grs::Twist<KDL::Vector, KDL::Vector> twist02 = grs::compose(twist_j2,twist01);
    std::cout << "L2 twist02 " << std::endl << twist02 << std::endl << std::endl;
    
    //~TWISTS
//~SEGMENT2

//SEGMENT3
    //SEGMENT METADATA
    // joint3 with respect to L2
    Joint joint3 = Joint("Segment3.Joint3", Vector(0.0, 0.0, 0),Vector(0,0,1),Joint::RotAxis, 1, 0, 0.01);
    grs::PoseCoordinatesSemantics pose_joint3_L2("j3","J3","Segment3.Joint3","l2","L2","Segment2.Link2","L2");
    Vector joint3_position3_L2 = joint3.JointOrigin();
    Rotation joint3_coord_orientation3_L2=joint3.pose(0).M;
    grs::PoseCoordinates<KDL::Frame> pose_coord_joint3_L2(KDL::Frame(joint3_coord_orientation3_L2, joint3_position3_L2));
    grs::Pose<KDL::Frame> posejoint3_L2(pose_joint3_L2, pose_coord_joint3_L2);

    //Link3 tip frame3
    grs::PoseCoordinatesSemantics pose_link3tip_L2("l3","L3","Segment3.Link3","l2","L2","Segment2.Link2","L2");
    Vector link3tip_position3_L2 = Vector(0.5, 0.0, 0);
    Rotation link3tip_coord_orientation3_L2 = Rotation::Identity();
    grs::PoseCoordinates<KDL::Frame> pose_coord_link3tip_L2(KDL::Frame(link3tip_coord_orientation3_L2, link3tip_position3_L2));
    grs::Pose<KDL::Frame> poselink3tip_L2(pose_link3tip_L2, pose_coord_link3tip_L2);
    Frame frame3 = poselink3tip_L2.getCoordinates().getCoordinates();
    Segment segment3 = Segment("Link3", joint3, frame3);
    //~SEGMENT METADATA
    
    //POSES
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
    //j3 on Segment3.Joint3 twist w.r.t Segment2.Link2
    grs::LinearVelocityCoordinatesSemantics linear_vel_coord_seman_j3("j3","Segment3.Joint3","Segment2.Link2","L2");
    KDL::Vector coordinatesLinearVelocity_j3 = joint3.twist(qdot(2)).vel;
    grs::LinearVelocity<KDL::Vector> linearVelocity_j3(linear_vel_coord_seman_j3 ,coordinatesLinearVelocity_j3);

    grs::AngularVelocityCoordinatesSemantics ang_vel_coord_seman_j3("Segment3.Joint3","Segment2.Link2","L2");
    KDL::Vector coordinatesAngularVelocity_j3 = joint3.twist(qdot(2)).rot;
    grs::AngularVelocity<KDL::Vector> angularVelocity_j3(ang_vel_coord_seman_j3, coordinatesAngularVelocity_j3);
    
    //joint.twist returns this.
    grs::Twist<KDL::Vector,KDL::Vector> twist_j3(linearVelocity_j3, angularVelocity_j3);
    
    // l1 on Segment1.Link1 twist w.r.t Base
    //distance between points j1 and l1 at changing value of q; at 0 it is pose_l1_j1_0.p
    // needs to be put in different coordinates with the same origin.
    grs::PositionCoordinates<KDL::Vector> distance3 = posejoint3_q_L2.getCoordinates().getCoordinates().M * (-1* pose_l3_j3_0.getCoordinates().getCoordinates().p);
    grs::Position<KDL::Vector> position_l3_l2_q("j3","Segment3.Joint3","l3", "Segment2.Link3", "L2", distance3);
    std::cout<< "distance " << distance3 << std::endl;
    
    grs::OrientationCoordinates<KDL::Rotation> orientation3 = pose_l3_l2_q.getCoordinates().getCoordinates().M.Inverse(); 
    grs::Orientation<KDL::Rotation> orientation_l3_j3_q("J2","Segment2.Joint2", "L3","Segment2.Link2","L2", orientation3);
    std::cout<< "orientation " << orientation3 << std::endl;
    
    y2 = twist_j3.changePointBody(position_l3_l2_q);
    std::cout << "Change reference point of joint twist " << std::endl << twist_j3 << std::endl;//segment.twist returns this. Segment tip twist with respect to previous segment tip
    x2 = twist_j3.changeCoordinateFrame(orientation_l3_j3_q);
    std::cout << "Change coordinate it is expressed in " << std::endl << twist_j3 << std::endl; //M.Inv(segment.twist) returns this. Segment tip twist with respect to joint frame
    
    //l1 on Segment1.Link1 twist w.r.t Joint1 expressed in L1
    //for the 2st link twist02 = twist01 + twist_j2 (in terms of values)
    t1 = twist02.changePointBody(position_l3_l2_q);
    if(t1)
        std::cout << "Change reference point of L2 twist02 " << std::endl << twist02 << std::endl;
    
    t2 = twist02.changeCoordinateFrame(orientation_l3_j3_q);
    if(t2)
        std::cout << "Change coordinate of L2 twist02 " << std::endl << twist02 << std::endl;
    
    grs::Twist<KDL::Vector, KDL::Vector> twist03 = grs::compose(twist_j3,twist02);
    std::cout << "L3 twist03 " << twist03 << std::endl << std::endl;
    
    //~TWISTS
    
//~SEGMENT3

    //FK
    grs::Pose<KDL::Frame> fk_pose_L3_B_q = grs::compose(pose_l1_b_q, grs::compose(pose_l2_l1_q,pose_l3_l2_q) );
    std::cout <<"Tip L3 with respect to B at value q  " << fk_pose_L3_B_q << std::endl;    
    cout << endl;  

    
    KDL::Chain achain;
    achain.addSegment(segment1);
    achain.addSegment(segment2);
    achain.addSegment(segment3);
    
    
    
    ChainFkSolverPos_recursive fksolvertest(achain);
    ChainFkSolverVel_recursive fkvelsolvertest(achain);
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
    
    for(unsigned int i=0; i<achain.getNrOfSegments();i++)
    {
//        std::cout <<"Segment joint name " << achain.getSegment(i).getJoint().getName() << std::endl;
//        std::cout <<"Segment name " << achain.getSegment(i).getName()<< std::endl;
//        std::cout <<"Joint origin " << achain.getSegment(i).getJoint().JointOrigin() << std::endl;
//        std::cout <<"Joint axis " << achain.getSegment(i).getJoint().JointAxis()<< std::endl;
        std::cout <<"Tip Frame " << achain.getSegment(i).getFrameToTip()<< std::endl;
        tempLocal[i]=achain.getSegment(i).pose(jointInitialPose(i));
        std::cout <<"Segment tip pose " << achain.getSegment(i).pose(jointInitialPose(i)) << std::endl;
//        std::cout <<"New reference point V_AB vector " << achain.getSegment(i).pose(jointInitialPose(i)).M *achain.getSegment(i).getFrameToTip().p << std::endl;
        std::cout <<"Joint twist " << achain.getSegment(i).getJoint().twist(jointInitialRate.qdot(i)) << std::endl;
        std::cout <<"Segment tip twist " << achain.getSegment(i).twist(jointInitialPose(i), jointInitialRate.qdot(i)) << std::endl;
        std::cout <<"Segment tip twist inv " << tempLocal[i].M.Inverse(achain.getSegment(i).twist(jointInitialPose(i), jointInitialRate.qdot(i))) << std::endl;
//        std::cout <<"Segment tip Unit twist inv " << tempLocal[i].M.Inverse(achain.getSegment(i).twist(jointInitialPose(i), 1.0)) << std::endl;
//        std::cout <<"Segment root Unit twist inv " << tempLocal[i]*tempLocal[i].M.Inverse(achain.getSegment(i).twist(jointInitialPose(i), 1.0)) << std::endl;
        if(i == 0)
        {
            tempTwist[i] = tempLocal[i].M.Inverse(achain.getSegment(i).twist(jointInitialPose(i), jointInitialRate.qdot(i)));
            std::cout <<"Total twist " << tempTwist[i] << std::endl;
        }
        if(i!=0)
        {       tempTwist[i] = tempLocal[i].Inverse(tempTwist[i-1]) + tempLocal[i].M.Inverse(achain.getSegment(i).twist(jointInitialPose(i), jointInitialRate.qdot(i))) ;
            std::cout <<"Total twist " << tempTwist[i] << std::endl;
        }
        fksolvertest.JntToCart(jointInitialPose, tempEEtest[i], i+1);
        std::cout << "FK Pose of link " << i+1 << " " << tempEEtest[i] << std::endl<< std::endl<< std::endl;
        fkvelsolvertest.JntToCart(jointInitialRate, tempVeltest[i], i+1);
        std::cout << "FK Vel of link " << i+1 << " " << tempVeltest[i].GetTwist() << std::endl<< std::endl<< std::endl;
    }
    
    return 0;
}