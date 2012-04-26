#include <frames.hpp>
#include <joint.hpp>
#include <frames_io.hpp>
#include <chainidsolver_constraint_vereshchagin.hpp>
#include <chainfksolver.hpp>
#include <chainfksolverpos_recursive.hpp>
#include <chainfksolvervel_recursive.hpp>

#include <chain.hpp>
#include <kinfam_io.hpp>

int main()
{

    using namespace KDL;

    //Definition of kinematic chain
    //-----------------------------------------------------------------------------------------------//
    //Joint (const JointType &type=None, const double &scale=1, const double &offset=0,
    //       const double &inertia=0, const double &damping=0, const double &stiffness=0)
    Joint rotJoint0 = Joint(Joint::RotZ, 1, 0, 0.01);
    Joint rotJoint1 = Joint(Joint::RotZ, 1, 0, 0.01);

    Frame refFrame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0));
    Frame frame1(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
    Frame frame2(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
    //Segment (const Joint &joint=Joint(Joint::None),
    //         const Frame &f_tip=Frame::Identity(), const RigidBodyInertia &I=RigidBodyInertia::Zero())
    Segment segment1 = Segment(rotJoint0, frame1);
    Segment segment2 = Segment(rotJoint1, frame2);

    // 	RotationalInertia (double Ixx=0, double Iyy=0, double Izz=0, double Ixy=0, double Ixz=0, double Iyz=0)
    RotationalInertia rotInerSeg1(0.0, 0.0, 0.0, 0.0, 0.0, 0.0); //around symmetry axis of rotation
    //RigidBodyInertia (double m=0, const Vector &oc=Vector::Zero(),
    //                    const RotationalInertia &Ic=RotationalInertia::Zero())
    RigidBodyInertia inerSegment1(0.3, Vector(0.0, -0.4, 0.0), rotInerSeg1);
    RigidBodyInertia inerSegment2(0.3, Vector(0.0, -0.4, 0.0), rotInerSeg1);
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
    Vector constrainXLinear(1.0, 0.0, 0.0);
    Vector constrainXAngular(0.0, 0.0, 0.0);
    Vector constrainYLinear(0.0, 0.0, 0.0);
    Vector constrainYAngular(0.0, 0.0, 0.0);
    const Twist constraintForcesX(constrainXLinear, constrainXAngular);
    //const Twist constraintForcesY(constrainYLinear, constrainYAngular);
    Jacobian alpha(1);
    alpha.setColumn(0, constraintForcesX);
    //alpha.setColumn(1, constraintForcesY);

    //Acceleration energy at  the end-effector
    JntArray betha(1); //set to zero
    betha(0) = 0.0;
    //betha(1) = 0.0;
    std::cout << "alpha " << alpha << std::endl;

    //arm root acceleration
    Vector linearAcc(0.0, 9.8, 0.0); //gravitational acceleration along Y
    //Vector linearAcc(0.0,0.0,0.0); //gravitational acceleration along Y
    Vector angularAcc(0.0, 0.0, 0.0);
    Twist twist1(linearAcc, angularAcc);

    std::cout << "root acceleration " << twist1.vel(1) << std::endl << std::endl;

    //external forces on the arm
    Vector externalForce(0.0, 0.0, 0.0);
    Vector externalTorque(0.0, 0.0, 0.0);
    Wrench externalNetForce1(externalForce, externalTorque);
    Wrench externalNetForce2(externalForce, externalTorque);
    Wrenches externalNetForce;
    externalNetForce.push_back(externalNetForce1);
    externalNetForce.push_back(externalNetForce2);
    //-------------------------------------------------------------------------------------//

    //solvers
    ChainFkSolverPos_recursive fksolver(chain);
    ChainFkSolverVel_recursive fksolverVel(chain);
    int x = 1;
    ChainIdSolver_Constraint_Vereshchagin constraintSolver(chain, twist1, x);
    bool status;

    //Defining cartesian space scheme constraints
    //This includes initial and final positions, velocities and acceleration constraints
    // Initial arm position configuration/constraint
    Frame frameInitialPose; //initial cartesian postion/orientation of the end effector
    JntArray jointInitialPose(chain.getNrOfJoints());
    jointInitialPose(0) = M_PI / 2.0; // initial joint0 pose
    jointInitialPose(1) = 0.0; //initial joint1 pose, negative in clockwise

    fksolver.JntToCart(jointInitialPose, frameInitialPose); //find initial cartesian pose from initial joint poses
    std::cout << "carPoseInitialX " << frameInitialPose.p.x() << std::endl;
    std::cout << "carPoseInitialY " << frameInitialPose.p.y() << std::endl;

    //final arm position constraint
    Frame frameFinalPose(Vector(frameInitialPose.p.x(), -0.5, 0)); //final postion of the end effector in Cartesian space
    JntArray qFinalPose(chain.getNrOfJoints()); //final positions of joints in joint space, joint values basically
    // cartesian Y pose changes according to
    //double cartYPose = a0 + a1*t + a2*t*t;
    Twist cartVelocityEE;

    Twist accLink0;
    Twist accLink1;
    //    Twists cartAcc;
    //    cartAcc.push_back(accLink0);
    //    cartAcc.push_back(accLink1);

    //arm joint rate configuration and initial values are zero
    JntArray qDot(chain.getNrOfJoints());
    //arm joint accelerations returned by the solver
    JntArray qDotDot(arrayOfJoints);
    qDot(0) = 5.0;
    //arm joint torques returned by the solver
    JntArray qTorque(arrayOfJoints);

    arrayOfJoints(0) = jointInitialPose(0);
    arrayOfJoints(1) = jointInitialPose(1);

    //Calculate joint values for each cartesian position
    Frame frameEEPose;
    Twist frameEEVel;
    double TimeConstant = 2.0; //Time required to complete the task move(frameinitialPose, framefinalPose)
    double timeDelta = 0.005;


    //cartesian space/link values
    //0-actual, 1-desire, 2-error, 3-errorsum
    Twist accLink;
    Twists cartXDotDot;
    cartXDotDot.push_back(accLink);
    cartXDotDot.push_back(accLink);

     Twists cartXDot;
    cartXDot.push_back(accLink);
    cartXDot.push_back(accLink);

    for (double t = 0.0; t <= TimeConstant; t = t + timeDelta)
    {
        /*
            cartVelocityEE.vel(0) = cartVelocityEE.vel(0) + cartAcc[1].vel(0)*t*0.5;
            frameEEPose.p[0] = frameInitialPose.p.x()+cartVelocityEE.vel(0)*t + cartAcc[1].vel(0)*t*t*0.5;
            cartVelocityEE.vel(1) = cartVelocityEE.vel(1) + cartAcc[1].vel(1)*t*0.5;
            frameEEPose.p[1] = frameInitialPose.p.y()+cartVelocityEE.vel(1)*t + cartAcc[1].vel(1)*t*t*0.5;
            std::cout << frameEEPose.p.x() << std::endl;
            std::cout << frameEEPose.p.y() << std::endl;
         */

        /*fksolver.JntToCart(arrayOfJoints,frameEEPose);

        std::cout << "X " << frameEEPose.p << std::endl;
        frameEEVel.vel(0) = 0.4*cos(arrayOfJoints(0))*qDot(0)+0.4*cos(arrayOfJoints(0)+arrayOfJoints(1))*(qDot(0)+qDot(1));
        frameEEVel.vel(1) = 0.4*sin(arrayOfJoints(0))*qDot(0)+0.4*sin(arrayOfJoints(0)+arrayOfJoints(1))*(qDot(0)+qDot(1));
        std::cout << "V " << frameEEVel.vel << std::endl;
        cartAcc[1].vel(1) =  (frameFinalPose.p.y()-frameEEPose.p.y()-50*t*frameEEVel.vel.y())/(t*t); */
        //at time t0, inputs are arrayOfjoints(0,pi/4), qdot(0,0)
        status = constraintSolver.CartToJnt(arrayOfJoints, qDot, qDotDot, cartXDotDot, alpha, betha, externalNetForce, qTorque);
        if (status >= 0)
        {
            std::cout << "q " << arrayOfJoints << std::endl << "qDot " << qDot << std::endl;
            std::cout << "qDotDot " << qDotDot << std::endl << "qTorque " << qTorque << std::endl;
            constraintSolver.getLinkCartesianVelocity(cartXDot);
            std::cout << "cartXDot" << cartXDot << std::endl;
        }

        //Integration at the given "instanteneous" interval for joint position and velocity.
        //These will be inputs for the next time instant
        //actually here it should be a time interval and not time from the beginning, that is why timeDelta;
        arrayOfJoints(0) = arrayOfJoints(0) + (qDot(0) + timeDelta * qDotDot(0) / 2.0) * timeDelta;
        qDot(0) = qDot(0) + qDotDot(0) * timeDelta;
        arrayOfJoints(1) = arrayOfJoints(1) + (qDot(1) + qDotDot(1) * timeDelta / 2.0) * timeDelta;
        qDot(1) = qDot(1) + qDotDot(1) * timeDelta;
    }

    //torque1=[(m1+m2)a1^2+m2a2^2+2m2a1a2cos2]qDotDot1+[m2a2^2+m2a1a2cos2]qDotDot2-m2a1a2(2qDot1qDot2+qDot2^2)sin2+(m1+m2)ga1cos2+m2ga2cos(1+2)

    //torque2=[m2a2^2+m2a1a2cos2]qDotDot1+m2a2^2qDotDot2+m2a1a2qDot1^2sin2+m2ga2cos(1+2)

    return 0;
}
