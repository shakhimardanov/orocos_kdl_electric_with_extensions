#include <frames.hpp>
#include <joint.hpp>
#include <frames_io.hpp>
#include <chainfksolverpos_recursive.hpp>
#include <chainidsolver_recursive_newton_euler.hpp>
#include <chain.hpp>
#include <kinfam_io.hpp>
#include <iostream>
#include <chainidsolver_vereshchagin.hpp>
//#include <chainidsolver_constraint_vereshchagin.hpp>
int main()
{

    using namespace KDL;


    //Definition of kinematic chain
    //-----------------------------------------------------------------------------------------------//
    //Joint (const JointType &type=None, const double &scale=1, const double &offset=0,
    //       const double &inertia=0, const double &damping=0, const double &stiffness=0)
    Joint rotJoint0 = Joint(Joint::RotZ, 1, 0, 0.01);
    Joint rotJoint1 = Joint(Joint::RotZ, 1, 0, 0.01);
    //Joint rotJoint1 = Joint(Joint::None,1,0,0.01);

    Frame refFrame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0));
    Frame frame1(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.2, 0.0));
    Frame frame2(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.2, 0.0));
    //Segment (const Joint &joint=Joint(Joint::None),
    //         const Frame &f_tip=Frame::Identity(), const RigidBodyInertia &I=RigidBodyInertia::Zero())
    Segment segment1 = Segment(rotJoint0, frame1);
    Segment segment2 = Segment(rotJoint1, frame2);

    // 	RotationalInertia (double Ixx=0, double Iyy=0, double Izz=0, double Ixy=0, double Ixz=0, double Iyz=0)
    RotationalInertia rotInerSeg1(0.0, 0.0, 0.0, 0.0, 0.0, 0.0); //around symmetry axis of rotation
    //RigidBodyInertia (double m=0, const Vector &oc=Vector::Zero(), const RotationalInertia &Ic=RotationalInertia::Zero())
    RigidBodyInertia inerSegment1(1.0, Vector(0.0, -0.2, 0.0), rotInerSeg1);
    RigidBodyInertia inerSegment2(1.0, Vector(0.0, -0.2, 0.0), rotInerSeg1);
    segment1.setInertia(inerSegment1);
    segment2.setInertia(inerSegment2);

    Chain chain;
    chain.addSegment(segment1);
    //chain.addSegment(segment2);
    //----------------------------------------------------------------------------------------------//

    //Definition of constraints and external disturbances
    //--------------------------------------------------------------------------------------//
    JntArray arrayOfJoints(chain.getNrOfJoints());
    //Constraint force matrix at the end-effector
    //What is the convention for the spatial force matrix; is it the same as in thesis?
    Vector constrainXLinear(0.0, 1.0, 0.0);
    Vector constrainXAngular(0.0, 0.0, 0.0);
    Vector constrainYLinear(0.0, 0.0, 0.0);
    Vector constrainYAngular(0.0, 0.0, 0.0);
    Vector constrainZLinear(0.0, 0.0, 0.0);
    Vector constrainZAngular(0.0, 0.0, 0.0);
    const Twist constraintForcesX(constrainXLinear, constrainXAngular);
    const Twist constraintForcesY(constrainYLinear, constrainYAngular);
    const Twist constraintForcesZ(constrainZLinear, constrainZAngular);
    Jacobian alpha(1);
    alpha.setColumn(0, constraintForcesX);
    //alpha.setColumn(1,constraintForcesY);
    //alpha.setColumn(2,constraintForcesZ);

    //Acceleration energy at  the end-effector
    JntArray betha(1); //set to zero
    betha(0) = 0.0;
    //betha(1) = 0.0;
    //betha(2) = 0.0;

    //arm root acceleration
    Vector linearAcc(0.0, 10, 0.0); //gravitational acceleration along Y
    Vector angularAcc(0.0, 0.0, 0.0);
    Twist twist1(linearAcc, angularAcc);

    //external forces on the arm
    Vector externalForce1(0.0, 0.0, 0.0);
    Vector externalTorque1(0.0, 0.0, 0.0);
    Vector externalForce2(0.0, 0.0, 0.0);
    Vector externalTorque2(0.0, 0.0, 0.0);
    Wrench externalNetForce1(externalForce1, externalTorque1);
    Wrench externalNetForce2(externalForce2, externalTorque2);
    Wrenches externalNetForce;
    externalNetForce.push_back(externalNetForce1);
    //externalNetForce.push_back(externalNetForce2);
    //-------------------------------------------------------------------------------------//

    //solvers
    ChainFkSolverPos_recursive fksolver(chain);
    //ChainFkSolverVel_recursive fksolverVel(chain);
    int numberOfConstraints = 1;
    ChainIdSolver_Vereshchagin constraintSolver(chain, twist1, numberOfConstraints);

    // Initial arm position configuration/constraint
    Frame frameInitialPose;
    JntArray jointInitialPose(chain.getNrOfJoints());
    jointInitialPose(0) = M_PI/6.0; // initial joint0 pose
    //jointInitialPose(1) = M_PI/6.0;                          //initial joint1 pose, negative in clockwise

    //cartesian space/link accelerations
    Twist accLink0;
    Twist accLink1;
    Twists cartAcc;
    cartAcc.push_back(accLink0);
    //cartAcc.push_back(accLink1);

    //arm joint rate configuration and initial values are zero
    JntArray qDot(chain.getNrOfJoints());
    //arm joint accelerations returned by the solver
    JntArray qDotDot(chain.getNrOfJoints());
    
    //arm joint torques returned by the solver
    JntArray qTorque(chain.getNrOfJoints());

    arrayOfJoints(0) = jointInitialPose(0);
    //arrayOfJoints(1) = jointInitialPose(1);

    //Calculate joint values for each cartesian position
    Frame frameEEPose;
    Twist frameEEVel;
    double TimeConstant = 1; //Time required to complete the task move(frameinitialPose, framefinalPose) default T=10.0
    double timeDelta = 0.002;
    bool status;

    Frame cartX1;
    Frame cartX2;
    Frames cartX;
    cartX.push_back(cartX1);
    cartX.push_back(cartX2);
    Twists cartXDot;
    cartXDot.push_back(accLink0);
    cartXDot.push_back(accLink1);

    Twists cartXDotDot;
    cartXDotDot.push_back(accLink0);
    cartXDotDot.push_back(accLink1);


    for (double t = 0.0; t <= 5*TimeConstant; t = t + timeDelta)
    {
        //at time t0, inputs are arrayOfjoints(0,pi/2.0), qdot(0,0)
        //qDot(0) = 0.5;
        // qDot(1) = 0.5;

        //printf("Inside the main loop %f  %f\n",qTorque(0), qTorque(1));
        

        //NOTE AZAMAT:what is qTorque? in implementation returned qTorque is constraint torque generated by the virtual constraint forces.
        // these torques are required to satify a virtual constraint and basically are real torques applied to joints to achieve
        // these constraint. But if additionally to the constraint generated torques we also had internal motor torques, then should not they
        //be added together?
        
        //qTorque(1) = qTorque(1)+internalJointTorque1;

        // In what frame of reference are the results expressed?

        status = constraintSolver.CartToJnt(arrayOfJoints, qDot, qDotDot, alpha, betha, externalNetForce, qTorque);
        if (status >= 0)
        {
            //For 2-dof arm
            //printf("%f          %f %f       %f %f       %f %f           %f %f           %f %f\n",t , arrayOfJoints(0), arrayOfJoints(1), qDot(0), qDot(1), qDotDot(0), qDotDot(1), qTorque(0), qTorque(1), frameEEPose.p.x(), frameEEPose.p.y());
            //printf("%f          %f %f       %f %f       %f %f\n", t, cartAcc[0].vel.x(), cartAcc[0].vel.y(), cartAcc[0].rot.z(), cartAcc[1].vel.x(), cartAcc[1].vel.y(), cartAcc[1].rot.z());

            //For 1-dof
            
            printf("Joint actual %f          %f       %f        %f            %f\n", t, arrayOfJoints(0), qDot(0), qDotDot(0), qTorque(0));
            std::cout<<std::endl;
        }
        constraintSolver.getLinkCartesianPose(cartX);
        constraintSolver.getLinkCartesianVelocity(cartXDot);
        constraintSolver.getLinkCartesianAcceleration(cartXDotDot);
        constraintSolver.getLinkAcceleration(cartAcc);
        printf("Cartesian acc local %f          %f       %f        %f\n", t, cartAcc[0].vel.x(), cartAcc[0].vel.y(), cartAcc[0].rot.z());
        printf("Cartesian actual %f          %f      %f     %f      %f\n", t, cartX[0].p.x(), cartX[0].p.y(), cartXDot[0].vel[0], cartXDot[0].vel[1]);
        std::cout<<std::endl;
        //printf("External torque %f\n", externalNetForce[0].torque.z());
        //Integration at the given "instanteneous" interval for joint position and velocity.
        //These will be inputs for the next time instant
        //actually here it should be a time interval and not time from the beginning, that is why timeDelta;
        arrayOfJoints(0) = arrayOfJoints(0) + (qDot(0) + qDotDot(0) * timeDelta / 2.0) * timeDelta;
        qDot(0) = qDot(0) + qDotDot(0) * timeDelta;
        //arrayOfJoints(1) = arrayOfJoints(1) + (qDot(1) + qDotDot(1)*timeDelta/2.0)*timeDelta;
        //qDot(1) = qDot(1)+qDotDot(1)*timeDelta;



        //For cartesian space control one needs to calculate from the obtained joint space value, new cartesian space poses.
        //Then based on the difference of the signal (desired-actual) we define a regulation function (controller)
        // this difference should be compensated either by constraint forces or by joint torques.
        // The current version of the algorithm assumes that we don't have control over torques, thus change in constraint forces
        //should be a valid option.
    }

    //torque1=[(m1+m2)a1^2+m2a2^2+2m2a1a2cos2]qDotDot1+[m2a2^2+m2a1a2cos2]qDotDot2-m2a1a2(2qDot1qDot2+qDot2^2)sin2+(m1+m2)ga1cos2+m2ga2cos(1+2)

    //torque2=[m2a2^2+m2a1a2cos2]qDotDot1+m2a2^2qDotDot2+m2a1a2qDot1^2sin2+m2ga2cos(1+2)

    return 0;
}
