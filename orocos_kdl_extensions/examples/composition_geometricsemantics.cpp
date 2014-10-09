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
    q(0)=M_PI/2.0;
    q(1)=M_PI/2.0;
    q(2)=M_PI/4.0;
    KDL::JntArray qdot(3);
    qdot(0)=0.15;
    qdot(1)=0.25;
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
    grs::Pose<KDL::Frame> pose_l1_j1_0 = compose(posejoint1_B.inverse2(), poselink1tip_B);
    // joint1 with respect to Base/World at some value q=M_PI/2.0    
    KDL::Rotation joint1_coord_orientation1_q_B = joint1.pose(q(0)).M;
    grs::PoseCoordinates<KDL::Frame> pose_coord_joint1_q_B(KDL::Frame(joint1_coord_orientation1_q_B, joint1_position1_B));
    grs::Pose<KDL::Frame> posejoint1_q_B(pose_joint1_B_semantics, pose_coord_joint1_q_B);
    //Link tip with respect to B while q is changing (segment.pose(q))
    grs::Pose<KDL::Frame> pose_l1_b_q = compose(posejoint1_q_B, pose_l1_j1_0);
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
    grs::PositionCoordinates<KDL::Vector> distance = pose_l1_j1_0.getCoordinates().getCoordinates().p;
    grs::Position<KDL::Vector> position_l1_j1_q("j1","Segment1.Joint1","l1", "Segment1.Link1", "J1", distance);
    
    
    grs::OrientationCoordinates<KDL::Rotation> orientation = posejoint1_q_B.getCoordinates().getCoordinates().M;
    grs::Orientation<KDL::Rotation> orientation_l1_j1_q("J1","Segment1.Joint1", "L1","Segment1.Link1","L1", orientation);
    
    bool y = twist_j1.changePointBody(position_l1_j1_q);
    std::cout << "Change reference point of joint twist " << twist_j1 << std::endl;//segment.twist returns this. Segment tip twist with respect to previous segment tip
    bool x = twist_j1.changeCoordinateFrame(orientation_l1_j1_q);
    std::cout << "Change coordinate it is expressed in " << twist_j1 << std::endl; //M.Inv(segment.twist) returns this. Segment tip twist with respect to joint frame
  
    
    grs::LinearVelocityCoordinatesSemantics linear_vel_coord_seman_l1("l1","Segment1.Link1","Base","B");
    KDL::Vector coordinatesLinearVelocity_l1(0,0,0);
    grs::LinearVelocity<Vector> linearVelocity_l1(linear_vel_coord_seman_l1, coordinatesLinearVelocity_l1);

    grs::AngularVelocityCoordinatesSemantics ang_vel_coord_seman_l1("Segment1.Link1","Base","B");
    Vector coordinatesAngularVelocity_l1(0,0,0);
    grs::AngularVelocity<Vector> angularVelocity_l1(ang_vel_coord_seman_l1, coordinatesAngularVelocity_l1);
    
    //segment.twist returns this. Segment tip twist with respect to previous segment tip
    grs::TwistCoordinatesSemantics twist_coord_sem_l1("l1","Segment1.Link1","Base","B"); 
    KDL::Twist coordinatesTwist_l1(coordinatesLinearVelocity_l1, coordinatesAngularVelocity_l1);
    grs::Twist<KDL::Twist> twist_l1(twist_coord_sem_l1, coordinatesTwist_l1);
    

    //l1 on Segment1.Link1 twist w.r.t Joint1
    grs::LinearVelocityCoordinatesSemantics linear_vel_coord_seman01("l1","Segment1.Link1","Segment1.Joint1","J1");
    Vector coordinatesLinearVelocity01(0,0,0);
    grs::LinearVelocity<Vector> linearVelocity01(linear_vel_coord_seman01 ,coordinatesLinearVelocity01);

    grs::AngularVelocityCoordinatesSemantics ang_vel_coord_seman01("Segment1.Link1","Segment1.Joint1","J1");
    Vector coordinatesAngularVelocity01(1,2,3);
    grs::AngularVelocity<Vector> angularVelocity01(ang_vel_coord_seman01, coordinatesAngularVelocity01);
    
    //this is joint angular vel contribution at the tip
    grs::TwistCoordinatesSemantics twist_coord_sem01("l1","Segment1.Link1","Segment1.Joint1","J1"); //M.Inv(segment.twist) returns this. Segment tip twist with respect to joint frame
    KDL::Twist coordinatesTwist01(coordinatesLinearVelocity01,coordinatesAngularVelocity01);
    grs::Twist<KDL::Twist> twist01(twist_coord_sem01, coordinatesTwist01);
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
    grs::Pose<KDL::Frame> pose_l2_j2_0 = compose(posejoint2_L1.inverse2() ,poselink2tip_L1);
    // joint2 with respect to L1 at some value q=M_PI/2.0    
    Rotation joint2_coord_orientation2_q_L1 = joint2.pose(q(1)).M;
    grs::PoseCoordinates<KDL::Frame> pose_coord_joint2_q_L1(KDL::Frame(joint2_coord_orientation2_q_L1, joint2_position2_L1));
    grs::Pose<KDL::Frame> posejoint2_q_L1(pose_joint2_L1_semantics, pose_coord_joint2_q_L1);
    //Link tip with respect to L1 while q is changing
    grs::Pose<KDL::Frame> pose_l2_l1_q = compose(posejoint2_q_L1, pose_l2_j2_0);
    if(pose_l2_l1_q.getCoordinates().getCoordinates() == segment2.pose(q(1)) )
    {
        std::cout <<"Tip L2 with respect to L1 at value q  " << pose_l2_l1_q << std::endl;    
        cout << endl;    
    }
    //~POSES
    
    //TWISTS
    //    //Note that joint2 and link2 belong to the same body/segment2
//    grs::LinearVelocityCoordinatesSemantics linear_vel_coord_seman2("l2","Segment2.Link2","Segment1.Link1","L1");
//    Vector coordinatesLinearVelocity2(1,2,3);
//    grs::LinearVelocity<Vector> linearVelocity2(linear_vel_coord_seman2 ,coordinatesLinearVelocity2);
//
//    grs::AngularVelocityCoordinatesSemantics ang_vel_coord_seman2("Link2","Segment1.Link1","L1");
//    Vector coordinatesAngularVelocity2(1,2,3);
//    grs::AngularVelocity<Vector> angularVelocity2(ang_vel_coord_seman2, coordinatesAngularVelocity2);
//
//    grs::TwistCoordinatesSemantics twist_coord_sem2("l2","Segment2.Link2","Segment1.Link1","L1"); //segment.twist returns this. Segment tip twist with respect to previous segment tip
//    KDL::Twist coordinatesTwist2(coordinatesLinearVelocity2,coordinatesAngularVelocity2);
//    grs::Twist<KDL::Twist> twist2(twist_coord_sem2, coordinatesTwist2);
//    
//    grs::LinearVelocityCoordinatesSemantics linear_vel_coord_seman02("l2","Segment2.Link2","Segment2.Joint2","J2");
//    Vector coordinatesLinearVelocity02(1,2,3);
//    grs::LinearVelocity<Vector> linearVelocity02(linear_vel_coord_seman02 ,coordinatesLinearVelocity02);
//
//    grs::AngularVelocityCoordinatesSemantics ang_vel_coord_seman02("Segment2.Link2","Segment2.Joint2","J2");
//    Vector coordinatesAngularVelocity02(1,2,3);
//    grs::AngularVelocity<Vector> angularVelocity02(ang_vel_coord_seman02, coordinatesAngularVelocity02);
//
//    grs::TwistCoordinatesSemantics twist_coord_sem02("l2","Segment2.Link2","Segment2.Joint2","J2"); //M.Inv(segment.twist) returns this. Segment tip twist with respect to joint frame
//    KDL::Twist coordinatesTwist02(coordinatesLinearVelocity02,coordinatesAngularVelocity02);
//    grs::Twist<KDL::Twist> twist02(twist_coord_sem02, coordinatesTwist02);
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
    grs::Pose<KDL::Frame> posejoint3_L2_withsemantics(pose_joint3_L2, pose_coord_joint3_L2);

    //Link3 tip frame3
    grs::PoseCoordinatesSemantics pose_link3tip_L2("l3","L3","Segment3.Link3","l2","L2","Segment2.Link2","L2");
    Vector link3tip_position3_L2 = Vector(0.5, 0.0, 0);
    Rotation link3tip_coord_orientation3_L2 = Rotation::Identity();
    grs::PoseCoordinates<KDL::Frame> pose_coord_link3tip_L2(KDL::Frame(link3tip_coord_orientation3_L2, link3tip_position3_L2));
    grs::Pose<KDL::Frame> poselink3tip_L2_withsemantics(pose_link3tip_L2, pose_coord_link3tip_L2);
    Frame frame3 = poselink3tip_L2_withsemantics.getCoordinates().getCoordinates();
    Segment segment3 = Segment("Link3", joint3, frame3);
    //~SEGMENT METADATA
    
    //POSES
    //Link3 tip with respect to joint3 at 0 defines the length of the segment
    grs::Pose<KDL::Frame> pose_l3_j3_0 = compose(posejoint3_L2_withsemantics.inverse2() ,poselink3tip_L2_withsemantics);
    // joint3 with respect to L2 at some value q=M_PI/2.0    
    Rotation joint3_coord_orientation3_q_L2=joint3.pose(q(2)).M;
    grs::PoseCoordinates<KDL::Frame> pose_coord_joint3_q_L2(KDL::Frame(joint3_coord_orientation3_q_L2, joint3_position3_L2));
    grs::Pose<KDL::Frame> posejoint3_q_L2_withsemantics(pose_joint3_L2, pose_coord_joint3_q_L2);
    //Link tip with respect to L2 while q is changing
    grs::Pose<KDL::Frame> pose_l3_l2_q = compose(posejoint3_q_L2_withsemantics, pose_l3_j3_0);
    if( pose_l3_l2_q.getCoordinates().getCoordinates() == segment3.pose(q(2)) )
    {
        std::cout <<"Tip L3 with respect to L2 at value q  " << pose_l3_l2_q << std::endl;    
        cout << endl;    
    }
    
    //FK
    grs::Pose<KDL::Frame> fk_pose_L3_B_q = compose(pose_l1_b_q, compose(pose_l2_l1_q,pose_l3_l2_q) );
    std::cout <<"Tip L3 with respect to B at value q  " << fk_pose_L3_B_q << std::endl;    
    cout << endl;  
    //~POSES
    
    //TWISTS
//    //Note that joint3 and link3 belong to the same body/segment3
//    grs::LinearVelocityCoordinatesSemantics linear_vel_coord_seman3("l3","Segment3.Link3","Segment2.Link2","L2");
//    Vector coordinatesLinearVelocity3(1,2,3);
//    grs::LinearVelocity<Vector> linearVelocity3(linear_vel_coord_seman3 ,coordinatesLinearVelocity3);
//
//    grs::AngularVelocityCoordinatesSemantics ang_vel_coord_seman3("Segment3.Link3","Segment2.Link2","L2");
//    Vector coordinatesAngularVelocity3(1,2,3);
//    grs::AngularVelocity<Vector> angularVelocity3(ang_vel_coord_seman3, coordinatesAngularVelocity3);
//
//    grs::TwistCoordinatesSemantics twist_coord_sem3("l3","Segment3.Link3","Segment2.Link2","L2"); //segment.twist returns this. Segment tip twist with respect to previous segment tip
//    KDL::Twist coordinatesTwist3(coordinatesLinearVelocity3,coordinatesAngularVelocity3);
//    grs::Twist<KDL::Twist> twist3(twist_coord_sem3, coordinatesTwist3);
//    
//    grs::LinearVelocityCoordinatesSemantics linear_vel_coord_seman03("l3","Segment3.Link3","Segment3.Joint3","J3");
//    Vector coordinatesLinearVelocity03(1,2,3);
//    grs::LinearVelocity<Vector> linearVelocity03(linear_vel_coord_seman03 ,coordinatesLinearVelocity03);
//
//    grs::AngularVelocityCoordinatesSemantics ang_vel_coord_seman03("Segment3.Link3","Segment3.Joint3","J3");
//    Vector coordinatesAngularVelocity03(1,2,3);
//    grs::AngularVelocity<Vector> angularVelocity03(ang_vel_coord_seman03, coordinatesAngularVelocity03);
//
//    grs::TwistCoordinatesSemantics twist_coord_sem03("l3","Segment3.Link3","Segment3.Joint3","J3"); //M.Inv(segment.twist) returns this. Segment tip twist with respect to joint frame
//    KDL::Twist coordinatesTwist03(coordinatesLinearVelocity03,coordinatesAngularVelocity03);
//    grs::Twist<KDL::Twist> twist03(twist_coord_sem03, coordinatesTwist03);
    //~TWISTS
    
//~SEGMENT3


    
//    KDL::Chain achain;
//    achain.addSegment(segment1);
//    achain.addSegment(segment2);
//    achain.addSegment(segment3);
//    
//    
//    
//    ChainFkSolverPos_recursive fksolvertest(achain);
//    Frame tempEEtest[3];  
//    KDL::JntArray jointInitialPose(achain.getNrOfJoints());
//    jointInitialPose(0)= M_PI/2.0;
//    jointInitialPose(1)=M_PI/2.0;
//    jointInitialPose(2)=M_PI/4.0;
//    
//    for(unsigned int i=0; i<achain.getNrOfSegments();i++)
//    {
//        std::cout <<"Segment joint name " << achain.getSegment(i).getJoint().getName() << std::endl;
//        std::cout <<"Segment name " << achain.getSegment(i).getName()<< std::endl;
//        std::cout <<"Joint origin " << achain.getSegment(i).getJoint().JointOrigin() << std::endl;
//        std::cout <<"Joint axis " << achain.getSegment(i).getJoint().JointAxis()<< std::endl;
//        std::cout <<"Tip Frame " << achain.getSegment(i).getFrameToTip()<< std::endl;
//        Frame tempLocal=achain.getSegment(i).pose(jointInitialPose(i));
//        std::cout <<"Segment tip pose " << achain.getSegment(i).pose(jointInitialPose(i)) << std::endl;
//        std::cout <<"Segment tip twist " << achain.getSegment(i).twist(jointInitialPose(i), 0.1) << std::endl;
//        std::cout <<"Segment tip twist inv" << tempLocal.M.Inverse(achain.getSegment(i).twist(jointInitialPose(i), 0.1)) << std::endl;
//        fksolvertest.JntToCart(jointInitialPose, tempEEtest[i], i+1);
//        std::cout << "FK of link " << i+1 << " " << tempEEtest[i] << std::endl<< std::endl<< std::endl;
//        
//    }
//    
    return 0;
}



void createMyTree(KDL::Tree& twoBranchTree)
{
    // joint1 with respect to Base/World
    Joint joint1 = Joint("Joint1", Vector(0.1, 0.0, 0.0),Vector(0,0,1),Joint::RotAxis, 1, 0, 0.01);
    grs::PoseCoordinatesSemantics pose_joint1_B("j1","J1","Joint1","b","B","Base","B");
    Vector joint1_position1_B = joint1.JointOrigin();
    Rotation joint1_coord_orientation1_B=joint1.pose(0).M;
    grs::PoseCoordinates<KDL::Frame> pose_coord_joint1_B(KDL::Frame(joint1_coord_orientation1_B, joint1_position1_B));
    grs::Pose<KDL::Frame> posejoint1_B_withsemantics(pose_joint1_B, pose_coord_joint1_B);
    
//    cout << "    pose_joint1_B =" << pose_joint1_B << endl;
//    cout << endl;
//    cout << "    posejoint1_B =" << posejoint1_B_withsemantics << endl;
//    cout << endl;
    
    //Link1 tip frame1 wrt B
    grs::PoseCoordinatesSemantics pose_link1tip_B("l1","L1","Link1","b","B","Base","B");
    Vector link1tip_position1_B = Vector(0.4, 0.0, 0.0);
    Rotation link1tip_coord_orientation1_B = Rotation::Identity();
    grs::PoseCoordinates<KDL::Frame> pose_coord_link1tip_B(KDL::Frame(link1tip_coord_orientation1_B, link1tip_position1_B));
    grs::Pose<KDL::Frame> poselink1tip_B_withsemantics(pose_link1tip_B, pose_coord_link1tip_B);
    Frame frame1 = poselink1tip_B_withsemantics.getCoordinates().getCoordinates();
    Segment segment1 = Segment("Link1", joint1, frame1);
    
    Joint rotJoint0 = Joint("joint0",Vector(-0.1, 0,0),Vector(0,0,1),Joint::RotAxis, 1, 0, 0.01);
    Frame frame0(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.4, 0.0, 0.0));
    Segment segment0 = Segment("segment0",rotJoint0, frame0);
    
//    cout << "    pose_link1_B =" << pose_link1tip_B << endl;
//    cout << endl;
//    cout << "    poselink1_B =" << poselink1tip_B_withsemantics << endl;
//    cout << endl;
    std::cout <<"Tip Frame1 " << segment0.getFrameToTip()<< std::endl;
    cout << endl;
    
    // joint2 with respect to L1
    Joint joint2 = Joint("Joint2", Vector(0.2, 0.0, 0),Vector(0,0,1),Joint::RotAxis, 1, 0, 0.01);
    grs::PoseCoordinatesSemantics pose_joint2_L1("j2","J2","Joint2","l1","L1","Link1","L1");
    Vector joint2_position2_L1 = joint2.JointOrigin();
    Rotation joint2_coord_orientation2_L1=joint2.pose(0).M;
    grs::PoseCoordinates<KDL::Frame> pose_coord_joint2_L1(KDL::Frame(joint2_coord_orientation2_L1, joint2_position2_L1));
    grs::Pose<KDL::Frame> posejoint2_L1_withsemantics(pose_joint2_L1, pose_coord_joint2_L1);
    
//    cout << "    pose_joint2_L1 =" << pose_joint2_L1 << endl;
//    cout << endl;
//    cout << "    posejoint2_L1 =" << posejoint2_L1_withsemantics << endl;
//    cout << endl;
    
    //Link2 tip frame2 w.r.t L1
    grs::PoseCoordinatesSemantics pose_link2tip_L1("l2","L2","Link2","l1","L1","Link1","L1");
    Vector link2tip_position2_L1 = Vector(0.4, 0.0, 0);
    Rotation link2tip_coord_orientation2_L1 = Rotation::Identity();
    grs::PoseCoordinates<KDL::Frame> pose_coord_link2tip_L1(KDL::Frame(link2tip_coord_orientation2_L1, link2tip_position2_L1));
    grs::Pose<KDL::Frame> poselink2tip_L1_withsemantics(pose_link2tip_L1, pose_coord_link2tip_L1);
    Frame frame2 = poselink2tip_L1_withsemantics.getCoordinates().getCoordinates();
    Segment segment2 = Segment("Link2", joint2, frame2);
    
//    cout << "    pose_link2_L1 =" << pose_link2tip_L1 << endl;
//    cout << endl;
//    cout << "    poselink2_L1 =" << poselink2tip_L1_withsemantics << endl;
//    cout << endl;
//    std::cout <<"Tip Frame2 " << segment2.getFrameToTip()<< std::endl;
//    cout << endl;    
    
    // joint3 with respect to L2
    Joint joint3 = Joint("Joint3", Vector(0.25, 0.0, 0),Vector(0,0,1),Joint::RotAxis, 1, 0, 0.01);
    grs::PoseCoordinatesSemantics pose_joint3_L2("j3","J3","Joint3","l2","L2","Link2","L2");
    Vector joint3_position3_L2 = joint3.JointOrigin();
    Rotation joint3_coord_orientation3_L2=joint3.pose(0).M;
    grs::PoseCoordinates<KDL::Frame> pose_coord_joint3_L2(KDL::Frame(joint3_coord_orientation3_L2, joint3_position3_L2));
    grs::Pose<KDL::Frame> posejoint3_L2_withsemantics(pose_joint3_L2, pose_coord_joint3_L2);
//    
//    cout << "    pose_joint3_L2 =" << pose_joint3_L2 << endl;
//    cout << endl;
//    cout << "    posejoint3_L2 =" << posejoint3_L2_withsemantics << endl;
//    cout << endl;
    
    //Link3 tip frame3
    grs::PoseCoordinatesSemantics pose_link3tip_L2("l3","L3","Link3","l2","L2","Link2","L2");
    Vector link3tip_position3_L2 = Vector(0.4, 0.0, 0);
    Rotation link3tip_coord_orientation3_L2 = Rotation::Identity();
    grs::PoseCoordinates<KDL::Frame> pose_coord_link3tip_L2(KDL::Frame(link3tip_coord_orientation3_L2, link3tip_position3_L2));
    grs::Pose<KDL::Frame> poselink3tip_L2_withsemantics(pose_link3tip_L2, pose_coord_link3tip_L2);
    Frame frame3 = poselink3tip_L2_withsemantics.getCoordinates().getCoordinates();
    Segment segment3 = Segment("Link3", joint3, frame3);
    
//    cout << "    pose_link3_L2 =" << pose_link3tip_L2 << endl;
//    cout << endl;
//    cout << "    poselink3_L2 =" << poselink3tip_L2_withsemantics << endl;
//    cout << endl;
//    std::cout <<"Tip Frame3 " << segment3.getFrameToTip()<< std::endl;
//    cout << endl;
    
    
    RotationalInertia rotInerSeg1(0.0, 0.0, 0.0, 0.0, 0.0, 0.0); //around symmetry axis of rotation
    double pointMass = 0.25; //in kg
    RigidBodyInertia inerSegment1(pointMass, Vector(0.4, 0.0, 0), rotInerSeg1);
    RigidBodyInertia inerSegment2(pointMass, Vector(0.4, 0.0, 0), rotInerSeg1);
    RigidBodyInertia inerSegment3(pointMass, Vector(0.4, 0.0, 0), rotInerSeg1);
    
    segment1.setInertia(inerSegment1);
    segment2.setInertia(inerSegment2);
    segment3.setInertia(inerSegment3);
    
    //Tree twoBranchTree("L0");
    twoBranchTree.addSegment(segment1, "L0");
    twoBranchTree.addSegment(segment2, "Link1");
    twoBranchTree.addSegment(segment3, "Link2");

}

void drawMyTree(KDL::Tree& twoBranchTree)
{

    //graphviz stuff
    /****************************************/
    Agraph_t *g;
    GVC_t *gvc;

    /* set up a graphviz context */
    gvc = gvContext();
    /* Create a simple digraph */
    g = agopen("robot-structure", AGDIGRAPH);

    //create vector to hold nodes
    std::vector<Agnode_t*> nodeVector;
    nodeVector.resize(twoBranchTree.getSegments().size());
    printf("size of segments in tree map %d\n", twoBranchTree.getSegments().size());
    printf("size of segments in tree %d\n", twoBranchTree.getNrOfSegments());

    //create vector to hold edges
    std::vector<Agedge_t*> edgeVector;
    edgeVector.resize(twoBranchTree.getNrOfJoints() + 1);
    int jointIndex = twoBranchTree.getNrOfJoints() + 1;
    printf("size of joint array %d %d\n", jointIndex, twoBranchTree.getNrOfJoints());

    int segmentIndex = 0;
    //    fill in the node vector by iterating over tree segments
    for (SegmentMap::const_iterator iter = twoBranchTree.getSegments().begin(); iter != twoBranchTree.getSegments().end(); ++iter)

    {
        //it would have been very useful if one could access list of joints of a tree
        //list of segments is already possible
        int stringLength = iter->second.segment.getName().size();
        char name[stringLength + 1];
        strcpy(name, iter->second.segment.getName().c_str());
        //q_nr returned is the same value for the root and the its child. this is a bug
        nodeVector[iter->second.q_nr] = agnode(g, name);
        agsafeset(nodeVector[iter->second.q_nr], "color", "red", "");
        agsafeset(nodeVector[iter->second.q_nr], "shape", "box", "");
        std::cout << "index parent " << iter->second.q_nr << std::endl;
        std::cout << "name parent " << iter->second.segment.getName() << std::endl;
        std::cout << "joint name parent " << iter->second.segment.getJoint().getName() << std::endl;
        std::cout << "joint type parent " << iter->second.segment.getJoint().getType() << std::endl;
        //        if (iter->second.segment.getJoint().getType() == Joint::None) //equals to joint type None
        //        {
        //            int stringLength = iter->second.segment.getJoint().getName().size();
        //            char name[stringLength + 1];
        //            strcpy(name, iter->second.segment.getJoint().getName().c_str());
        //            edgeVector[iter->second.q_nr] = agedge(g, nodeVector[iter->second.q_nr], nodeVector[iter->second.q_nr]);
        //            agsafeset(edgeVector[iter->second.q_nr], "label", name, "");
        //        }
        if (segmentIndex < twoBranchTree.getSegments().size())
            segmentIndex++;

    }

    //fill in edge vector by iterating over joints in the tree
    for (SegmentMap::const_iterator iter = twoBranchTree.getSegments().begin(); iter != twoBranchTree.getSegments().end(); ++iter)

    {
        //TODO: Fix node-edge connection relation
        int stringLength = iter->second.segment.getJoint().getName().size();
        std::cout << "Joint name " << iter->second.segment.getJoint().getName() << std::endl;
        char name[stringLength + 1];
        strcpy(name, iter->second.segment.getJoint().getName().c_str());
        for (std::vector<KDL::SegmentMap::const_iterator>::const_iterator childIter = iter->second.children.begin(); childIter != iter->second.children.end(); childIter++)
        {
            edgeVector[iter->second.q_nr] = agedge(g, nodeVector[iter->second.q_nr], nodeVector[(*childIter)->second.q_nr]);
            agsafeset(edgeVector[iter->second.q_nr], "label", name, "");
        }

        //            if (jointIndex != 0)
        //            {
        //                edgeVector[jointIndex] = agedge(g, nodeVector[segmentIndex], nodeVector[jointIndex]);
        //                agsafeset(edgeVector[jointIndex], "label", name, "");
        //            }

    }



    /* Compute a layout using layout engine from command line args */
    //  gvLayoutJobs(gvc, g);
    gvLayout(gvc, g, "dot");

    /* Write the graph according to -T and -o options */
    //gvRenderJobs(gvc, g);
    gvRenderFilename(gvc, g, "ps", "test-fext.ps");

    /* Free layout data */
    gvFreeLayout(gvc, g);

    /* Free graph structures */
    agclose(g);

    gvFreeContext(gvc);
    /* close output file, free context, and return number of errors */
    return;
}



void computeRNEDynamicsForChain(KDL::Tree& twoBranchTree, const std::string& rootLink, const std::string& tipLink, KDL::Vector& grav,
                                std::vector<kdle::JointState>& jointState, std::vector<kdle::SegmentState>& linkState)
{
    KDL::Chain achain;

    twoBranchTree.getChain(rootLink, tipLink, achain);
    KDL::ChainIdSolver_RNE *rneDynamics = new ChainIdSolver_RNE(achain, grav);


    KDL::JntArray q(achain.getNrOfJoints());
    KDL::JntArray q_dot(achain.getNrOfJoints());
    KDL::JntArray q_dotdot(achain.getNrOfJoints());
    JntArray torques(achain.getNrOfJoints());
    
    KDL::Wrenches f_ext;
    f_ext.resize(achain.getNrOfSegments());
    
    Vector forceComponent(-2.0, 2.0, 0.0);
//    Vector forceComponent(0.0, 0.0, 0.0);
    Vector torqueComponent(0.0, 0.0, 0.0);
    Wrench extForceLastSegment(forceComponent, torqueComponent);

    f_ext[achain.getNrOfSegments()-1] = extForceLastSegment;
    std::cout << endl << endl << endl;
    printf("RNE dynamics values \n");
    std::cout << f_ext[achain.getNrOfSegments()-1] << std::endl;

    for (unsigned int i = 0; i < achain.getNrOfJoints(); ++i)
    {
        q(i) = jointState[i].q;
        q_dot(i) = jointState[i].qdot;
        q_dotdot(i) = jointState[i].qdotdot;
        printf("q, qdot %f, %f\n", q(i), q_dot(i));
    }

    rneDynamics->CartToJnt(q, q_dot, q_dotdot, f_ext, torques);

    for (unsigned int i = 0; i < achain.getNrOfJoints(); ++i)
    {
        printf("index, q, qdot, torques %d, %f, %f, %f\n", i, q(i), q_dot(i), torques(i));
    }
    return;
}

void computeTemplatedDynamicsForTree(KDL::Tree& twoBranchTree, KDL::Vector& grav, std::vector<kdle::JointState>& jointState,
                                     std::vector<kdle::SegmentState>& linkState, std::vector<kdle::SegmentState>& linkState2)
{
    printf("Templated dynamics values for Tree \n");
    kdle::transform<tree_iterator, pose> _comp1;
    kdle::transform<tree_iterator, twist> _comp2;
    kdle::transform<tree_iterator, accTwist> _comp3;
    kdle::balance<tree_iterator, force> _comp4;
    kdle::project<tree_iterator,wrench> _comp5;
    
#ifdef VERBOSE_CHECK_MAIN
    std::cout << "Transform initial state" << std::endl << linkState[0].X << std::endl;
    std::cout << "Twist initial state" << std::endl << linkState[0].Xdot << std::endl;
    std::cout << "Acc Twist initial state" << std::endl << linkState[0].Xdotdot << std::endl;
    std::cout << "Wrench initial state" << std::endl << linkState[0].F << std::endl << std::endl;
#endif

#ifdef VERBOSE_CHECK_MAIN
    std::cout << "Transform L1" << linkState[1].X << std::endl;
    std::cout << "Twist L1" << linkState[1].Xdot << std::endl;
    std::cout << "Acc Twist L1" << linkState[1].Xdotdot << std::endl;
    std::cout << "Wrench L1" << linkState[1].F << std::endl << std::endl;
#endif

#ifdef VERBOSE_CHECK_MAIN
    std::cout << "Transform L2" << linkState[2].X << std::endl;
    std::cout << "Twist L2" << linkState[2].Xdot << std::endl;
    std::cout << "Acc Twist L2" << linkState[2].Xdotdot << std::endl;
    std::cout << "Wrench L2" << linkState[2].F << std::endl << std::endl;
#endif

    //typedef Composite<kdle::func_ptr(myTestComputation), kdle::func_ptr(myTestComputation) > compositeType0;
    typedef Composite< kdle::transform<tree_iterator, twist>, kdle::transform<tree_iterator, pose> > compositeType1;
    typedef Composite< kdle::balance<tree_iterator, force>, kdle::transform<tree_iterator, accTwist> > compositeType2;
    typedef Composite<compositeType2, compositeType1> compositeType3;

//    compositeType1 composite1 = kdle::compose(_comp2, _comp1);
    compositeType3 composite2 = kdle::compose(kdle::compose(_comp4, _comp3), kdle::compose(_comp2, _comp1));

    //kdle::DFSPolicy<KDL::Tree> mypolicy;
    kdle::DFSPolicy_ver2<KDL::Tree, inward> mypolicy1;
    kdle::DFSPolicy_ver2<KDL::Tree, outward> mypolicy2;

    std::cout << std::endl << std::endl << "FORWARD TRAVERSAL" << std::endl << std::endl;

//    traverseGraph_ver2(twoBranchTree, composite2, mypolicy2)(jointState, jointState, linkState, linkState2);
     traverseGraph_ver2(twoBranchTree, composite2, mypolicy2)(jointState, linkState, linkState2);

#ifdef VERBOSE_CHECK_MAIN
    std::cout << std::endl << std::endl << "LSTATE" << std::endl << std::endl;
    for (unsigned int i = 0; i < twoBranchTree.getNrOfSegments(); i++)
    {
        std::cout << linkState[i].segmentName << std::endl;
        std::cout << std::endl << linkState[i].X << std::endl;
        std::cout << linkState[i].Xdot << std::endl;
        std::cout << linkState[i].Xdotdot << std::endl;
        std::cout << linkState[i].F << std::endl;
    }
    std::cout << std::endl << std::endl << "LSTATE2" << std::endl << std::endl;
     for (unsigned int i = 0; i < twoBranchTree.getNrOfSegments(); i++)
    {
        std::cout << linkState2[i].segmentName << std::endl;
        std::cout << std::endl << linkState2[i].X << std::endl;
        std::cout << linkState2[i].Xdot << std::endl;
        std::cout << linkState2[i].Xdotdot << std::endl;
        std::cout << linkState2[i].F << std::endl;
    }
#endif

    std::vector<kdle::SegmentState> linkState3;
    linkState3.resize(twoBranchTree.getNrOfSegments()+1);
    std::cout << std::endl << std::endl << "REVERSE TRAVERSAL" << std::endl << std::endl;
    std::vector<kdle::JointState> jstate1;
    jstate1.resize(twoBranchTree.getNrOfSegments() + 1);
    traverseGraph_ver2(twoBranchTree, _comp5, mypolicy1)(jointState, jstate1, linkState2, linkState3);
    //version 1 traversal
    //traverseGraph(twoBranchTree, kdl_extensions::func_ptr(myTestComputation), mypolicy)(1, 2, 3);    
#ifdef VERBOSE_CHECK_MAIN
    std::cout << std::endl << std::endl << "LSTATE3" << std::endl << std::endl;
    for (KDL::SegmentMap::const_reverse_iterator iter = twoBranchTree.getSegments().rbegin(); iter != twoBranchTree.getSegments().rend(); ++iter)
    {
        std::cout << std::endl << iter->first << std::endl;
        std::cout << linkState3[iter->second.q_nr].X << std::endl;
        std::cout << linkState3[iter->second.q_nr].Xdot << std::endl;
        std::cout << linkState3[iter->second.q_nr].Xdotdot << std::endl;
        std::cout << linkState3[iter->second.q_nr].F << std::endl;
        std::cout << "Joint index and torque " << iter->second.q_nr << "  " << jstate1[iter->second.q_nr].torque << std::endl;
    }
#endif
    return;
}