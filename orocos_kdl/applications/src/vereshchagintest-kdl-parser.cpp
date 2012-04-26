#include <frames.hpp>
#include <joint.hpp>
#include <frames_io.hpp>
#include <chainidsolver_constraint_vereshchagin.hpp>
#include <chainfksolverpos_recursive.hpp>
#include <chainidsolver_recursive_newton_euler.hpp>
#include <chain.hpp>
#include <kinfam_io.hpp>
//#include <kdl_parser/kdl_parser.hpp>

int main()
{

    using namespace KDL;
/*
    KDL::Tree my_tree;
   if (!kdl_parser::treeFromFile("planar2RArm.xml", my_tree)){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
   }
*/



//Definition of kinematic chain
//-----------------------------------------------------------------------------------------------//
    //Joint (const JointType &type=None, const double &scale=1, const double &offset=0,
    //       const double &inertia=0, const double &damping=0, const double &stiffness=0)
    Joint rotJoint0 = Joint(Joint::RotZ,1,0,0.01);
    Joint rotJoint1 = Joint(Joint::RotZ,1,0,0.01);

    Frame refFrame(Rotation::RPY(0.0,0.0,0.0), Vector(0.0,0.0,0.0) );
    Frame frame1(Rotation::RPY(0.0,0.0,0.0), Vector(0.0,-0.4,0.0) );
    Frame frame2(Rotation::RPY(0.0,0.0,0.0), Vector(0.0,-0.4,0.0) );
    //Segment (const Joint &joint=Joint(Joint::None),
    //         const Frame &f_tip=Frame::Identity(), const RigidBodyInertia &I=RigidBodyInertia::Zero())
    Segment segment1 = Segment(rotJoint0, frame1);
    Segment segment2 = Segment(rotJoint1, frame2);

    // 	RotationalInertia (double Ixx=0, double Iyy=0, double Izz=0, double Ixy=0, double Ixz=0, double Iyz=0)
    RotationalInertia rotInerSeg1(0.0,0.0,0.0,0.0,0.0,0.0); //around symmetry axis of rotation
    //RigidBodyInertia (double m=0, const Vector &oc=Vector::Zero(), const RotationalInertia &Ic=RotationalInertia::Zero())
    RigidBodyInertia  inerSegment1(0.3,Vector(0.0,-0.4,0.0),rotInerSeg1);
    RigidBodyInertia  inerSegment2(0.3,Vector(0.0,-0.4,0.0),rotInerSeg1);
    segment1.setInertia(inerSegment1);
    segment2.setInertia(inerSegment2);

    Chain chain;
    chain.addSegment(segment1);
    chain.addSegment(segment2);
//----------------------------------------------------------------------------------------------//

//Definition of constraints and external disturbances
//--------------------------------------------------------------------------------------//
    JntArray arrayOfJoints(chain.getNrOfJoints());
    //Constraint force matrix at the end-effector
    //What is the convention for the spatial force matrix; is it the same as in thesis?
    Vector constrainXLinear(0.0,0.0,0.0);
    Vector constrainXAngular(0.0,0.0,0.0);
    Vector constrainYLinear(0.0,0.0,0.0);
    Vector constrainYAngular(0.0,0.0,0.0);
 //   Vector constrainZLinear(0.0,0.0,0.0);
  //  Vector constrainZAngular(0.0,0.0,0.0);
    const Twist constraintForcesX(constrainXLinear,constrainXAngular);
    const Twist constraintForcesY(constrainYLinear,constrainYAngular);
 //   const Twist constraintForcesZ(constrainZLinear,constrainZAngular);
    Jacobian alpha(2);
    alpha.setColumn(0,constraintForcesX);
    alpha.setColumn(1,constraintForcesY);
 //   alpha.setColumn(2,constraintForcesZ);
    //std::cout << "alpha "<< alpha << std::endl;

    //Acceleration energy at  the end-effector
    JntArray betha(2); //set to zero
    betha(0) = 0.0;
    betha(1) = 0.0;
  //  betha(2) = 0.0;
    std::cout << "alpha "<< alpha << std::endl;
    //std::cout << "betha "<< betha << std::endl;

    //arm root acceleration
    Vector linearAcc(0.0,9.8,0.0); //gravitational acceleration along -Y
    Vector angularAcc(0.0,0.0,0.0);
    Twist twist1(linearAcc,angularAcc);

    std::cout << "root acceleration " << twist1.vel(1) << std::endl << std::endl;

    //external forces on the arm
    Vector externalForce(0.0,0.0,0.0);
    Vector externalTorque(0.0,0.0,0.0);
    Vector externalTorque2(0.0,0.0,0.0);
    Wrench externalNetForce1(externalForce,externalTorque);
    Wrench externalNetForce2(externalForce,externalTorque2);
    Wrenches externalNetForce;
    externalNetForce.push_back(externalNetForce1);
    externalNetForce.push_back(externalNetForce2);
//-------------------------------------------------------------------------------------//

    //solvers
    ChainFkSolverPos_recursive fksolver(chain);
    //ChainFkSolverVel_recursive fksolverVel(chain);
    int constraints = 2;
    ChainIdSolver_Constraint_Vereshchagin constraintSolver(chain, twist1, constraints);

    // Initial arm position configuration/constraint
    Frame frameInitialPose;
    JntArray jointInitialPose(chain.getNrOfJoints());
    jointInitialPose(0) = M_PI/4.0;                      // initial joint0 pose
    jointInitialPose(1) = 0.0;                          //initial joint1 pose, negative in clockwise

    fksolver.JntToCart(jointInitialPose,frameInitialPose);  //find initial cartesian pose from initial joint poses
    std::cout << "carPoseInitialX " << frameInitialPose.p.x() << std::endl;
    std::cout << "carPoseInitialY " << frameInitialPose.p.y() << std::endl;

    //final arm position constraint
    Frame frameFinalPose(Vector(frameInitialPose.p.x(),-0.5,0));   //final postion of the end effector in Cartesian space
    JntArray qFinalPose(chain.getNrOfJoints());                    //final positions of joints in joint space, joint values basically

    //cartesian space/link accelerations
    Twist accLink0;
    Twist accLink1;
    Twists cartAcc;
    cartAcc.push_back(accLink0);
    cartAcc.push_back(accLink1);

    //arm joint rate configuration and initial values are zero
    JntArray qDot(chain.getNrOfJoints());
    //arm joint accelerations returned by the solver
    JntArray qDotDot(chain.getNrOfJoints());
    //arm joint torques returned by the solver
    JntArray qTorque(chain.getNrOfJoints());

    arrayOfJoints(0) = jointInitialPose(0);
    arrayOfJoints(1) = jointInitialPose(1);

    //Calculate joint values for each cartesian position
    Frame frameEEPose;
    Twist frameEEVel;
    double TimeConstant = 2;    //Time required to complete the task move(frameinitialPose, framefinalPose)
    double timeDelta = 0.025;
    bool status;

    for(double t=0.0; t<=TimeConstant; t=t+timeDelta)
    {
        //at time t0, inputs are arrayOfjoints(0,pi/4), qdot(0,0)
        fksolver.JntToCart(arrayOfJoints,frameEEPose);

        status = constraintSolver.CartToJnt(arrayOfJoints,qDot,qDotDot, cartAcc, alpha, betha, externalNetForce, qTorque);
        if(status >= 0)
        {
            printf("Time                  Joint pose                   qDot                      qDotDot                        qTorque                     X pose\n");
            printf("%f          %f %f       %f %f       %f %f           %f %f           %f %f\n",t , arrayOfJoints(0), arrayOfJoints(1), qDot(0), qDot(1), qDotDot(0), qDotDot(1), qTorque(0), qTorque(1), frameEEPose.p.x(), frameEEPose.p.y());

        }

        printf("External torque %f\n", externalNetForce[0].torque.z());
        //Integration at the given "instanteneous" interval for joint position and velocity.
        //These will be inputs for the next time instant
        //actually here it should be a time interval and not time from the beginning, that is why timeDelta;
        arrayOfJoints(0) = arrayOfJoints(0) + (qDot(0) + qDotDot(0)*timeDelta/2.0)*timeDelta;
        qDot(0) =  qDot(0)+qDotDot(0)*timeDelta;
        arrayOfJoints(1) = arrayOfJoints(1) + (qDot(1) + qDotDot(1)*timeDelta/2.0)*timeDelta;
        qDot(1) = qDot(1)+qDotDot(1)*timeDelta;
    }

//torque1=[(m1+m2)a1^2+m2a2^2+2m2a1a2cos2]qDotDot1+[m2a2^2+m2a1a2cos2]qDotDot2-m2a1a2(2qDot1qDot2+qDot2^2)sin2+(m1+m2)ga1cos2+m2ga2cos(1+2)

//torque2=[m2a2^2+m2a1a2cos2]qDotDot1+m2a2^2qDotDot2+m2a1a2qDot1^2sin2+m2ga2cos(1+2)

    return 0;
}
