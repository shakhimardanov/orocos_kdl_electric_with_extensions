#include <frames.hpp>
#include <joint.hpp>
#include <frames_io.hpp>
#include <chainidsolver_vereshchagin.hpp>
#include <chain.hpp>
#include <kinfam_io.hpp>
#include <iostream>

int main()
{

    using namespace KDL;


    //Definition of kinematic chain
    //-----------------------------------------------------------------------------------------------//
    //Joint (const JointType &type=None, const double &scale=1, const double &offset=0, const double &inertia=0, const double &damping=0, const double &stiffness=0)
    Joint rotJoint0 = Joint(Joint::RotZ, 1, 0, 0.01);
    Joint rotJoint1 = Joint(Joint::RotZ, 1, 0, 0.01);

    Frame refFrame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0));
    Frame frame1(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
    Frame frame2(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, -0.4, 0.0));
    //Segment (const Joint &joint=Joint(Joint::None), const Frame &f_tip=Frame::Identity(), const RigidBodyInertia &I=RigidBodyInertia::Zero())
    Segment segment1 = Segment(rotJoint0, frame1);
    Segment segment2 = Segment(rotJoint1, frame2);

    // 	RotationalInertia (double Ixx=0, double Iyy=0, double Izz=0, double Ixy=0, double Ixz=0, double Iyz=0)
    RotationalInertia rotInerSeg1(0.0, 0.0, 0.0, 0.0, 0.0, 0.0); //around symmetry axis of rotation
    //RigidBodyInertia (double m=0, const Vector &oc=Vector::Zero(), const RotationalInertia &Ic=RotationalInertia::Zero())
    RigidBodyInertia inerSegment1(0.3, Vector(0.0, -0.4, 0.0), rotInerSeg1);
    RigidBodyInertia inerSegment2(0.3, Vector(0.0, -0.4, 0.0), rotInerSeg1);
    segment1.setInertia(inerSegment1);
    segment2.setInertia(inerSegment2);

    Chain chain;
    chain.addSegment(segment1);
    chain.addSegment(segment2);
    //~Definition of kinematic chain
    //----------------------------------------------------------------------------------------------//



    //Definition of constraints and external disturbances
    //--------------------------------------------------------------------------------------//

    //Constraint force matrix at the end-effector
    //What is the convention for the spatial force matrix; is it the same as in thesis?
    //NOTE AZAMAT: Constraint are also defined in local reference frame?!
    Vector constrainXLinear(1.0, 0.0, 0.0);
    Vector constrainXAngular(0.0, 0.0, 0.0);
    Vector constrainYLinear(0.0, 0.0, 0.0);
    Vector constrainYAngular(0.0, 0.0, 0.0);
    Vector constrainZLinear(0.0, 0.0, 0.0);
    Vector constrainZAngular(0.0, 0.0, 0.0);
    Twist constraintForcesX(constrainXLinear, constrainXAngular);
    Twist constraintForcesY(constrainYLinear, constrainYAngular);
    Twist constraintForcesZ(constrainZLinear, constrainZAngular);
    Jacobian alpha(3);
    alpha.setColumn(0, constraintForcesX);
    alpha.setColumn(1, constraintForcesY);
    alpha.setColumn(2, constraintForcesZ);

    //Acceleration energy at  the end-effector
    JntArray betha(3); //set to zero
    betha(0) = 0.0;
    betha(1) = 0.0;
    betha(2) = 0.0;

    //arm root acceleration
    Vector linearAcc(0.0, 9.8, 0.0); //gravitational acceleration along Y
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
    externalNetForce.push_back(externalNetForce2);
    //~Definition of constraints and external disturbances
    //-------------------------------------------------------------------------------------//


    //Definition of solver and initial configuration
    //-------------------------------------------------------------------------------------//
    int numberOfConstraints = 3;
    ChainIdSolver_Vereshchagin constraintSolver(chain, twist1, numberOfConstraints);

    //These arrays of joint values contains values for 3
    //consequetive steps
    int k = 3;
    JntArray jointPoses[k];
    JntArray jointRates[k];
    JntArray jointAccelerations[k];
    JntArray jointTorques[k];
    JntArray biasqDotDot(chain.getNrOfJoints());
    for (int i = 0; i < k; i++)
    {
        JntArray jointValues(chain.getNrOfJoints());
        jointPoses[i] = jointValues;
        jointRates[i] = jointValues;
        jointAccelerations[i] = jointValues;
        jointTorques[i] = jointValues;
    }


    //cartesian space/link values
    Frames cartX[k];
    Twists cartXDot[k];
    Twists cartXDotDot[k];
    Twist accLink;
    for (int i = 0; i < k; i++) //i is number of variables (actual, desired, error)
    {
        for (int j = 0; j < k - 1; j++) //j is number of links
        {
            cartX[i].push_back(frame1);
            cartXDot[i].push_back(accLink);
            cartXDotDot[i].push_back(accLink);
        }

    }

    //at time=0.0 positions, rates and accelerations
    JntArray jointInitialPose(chain.getNrOfJoints());
    jointInitialPose(0) = 0.0; // initial joint0 pose
    jointInitialPose(1) = M_PI / 6.0; //initial joint1 pose, negative in clockwise
    //j0=0.0, j1=pi/6.0 correspond to x = 0.2, y = -0.7464
    //j0=pi/6.0, j1=pi/6.0 correspond to x = 0.5464, y = -0.5464
    //~Definition of solver and initial configuration
    //-------------------------------------------------------------------------------------//

    //First iteration to kick start integration process, because we need 0,1 iterations for the integration process
    //We use Euler method for the iteration 0
    double timeDelta = 0.001; //sampling interval/step size
    double TimeConstant = 60; //Time required to complete the task
    double Kp = 150.0;
    double Kv = 10.0;

    //Interpolator parameters:
    double b0_y = -0.7464102; //should come from initial joint configuration
    double b1_y = 0.0;
    double b2_y = 0.0; //((0.5 + 0.7464)*3.0 / TimeConstant * TimeConstant);
    double b3_y = 0.0; //-((0.5 + 0.7464)*2.0 / TimeConstant * TimeConstant * TimeConstant);

    double b0_x = 0.2; //should come from initial joint configuration
    double b1_x = 0.0;
    double b2_x = 0.0; //((0.5 + 0.7464)*3.0 / TimeConstant * TimeConstant);
    double b3_x = 0.0; //-((0.5 + 0.7464)*2.0 / TimeConstant * TimeConstant * TimeConstant);

    //Desired
    cartX[1][1].p[0] = b0_x;
    cartX[1][1].p[1] = b0_y;
    cartXDot[1][1].vel[0] = 0.0;
    cartXDot[1][1].vel[1] = 0.0;
    cartXDotDot[1][1].vel[0] = 0.0;
    cartXDotDot[1][1].vel[1] = 0.0;
    
    //step 0
    jointPoses[0](0) = jointInitialPose(0);
    jointPoses[0](1) = jointInitialPose(1);

    constraintSolver.CartToJnt(jointPoses[0], jointRates[0], jointAccelerations[0], cartXDotDot[0], alpha, betha, externalNetForce, jointTorques[0]);
    //Actual
    constraintSolver.getLinkCartesianPose(cartX[0]);
    constraintSolver.getLinkCartesianVelocity(cartXDot[0]);
    
    //Error
    cartX[2][1].p[0] = cartX[1][1].p[0] - cartX[0][1].p[0];
    cartX[2][1].p[1] = cartX[1][1].p[1] - cartX[0][1].p[1];
    cartXDot[2][1].vel[0] = cartXDot[1][1].vel[0] - cartXDot[0][1].vel[0];
    cartXDot[2][1].vel[1] = cartXDot[1][1].vel[1] - cartXDot[0][1].vel[1];
    cartXDotDot[2][1].vel[0] = cartXDotDot[1][1].vel[0] - cartXDotDot[0][1].vel[0];
    cartXDotDot[2][1].vel[1] = cartXDotDot[1][1].vel[1] - cartXDotDot[0][1].vel[1];
    printf("Iter 0:   %f      %f     %f         %f        %f      %f\n", cartX[2][1].p[0], cartX[2][1].p[1], cartXDot[2][1].vel[0], cartXDot[2][1].vel[1], cartXDotDot[2][1].vel[1], cartXDotDot[2][1].vel[1]);

    //Regulator or controller
    betha(0) = alpha(0, 0)*((Kp / 10000.0) * cartXDotDot[2][1].vel[0] + (Kv / 200.0) * cartXDot[2][1].vel[0] + (2 * Kp) * cartX[2][1].p[0]);
    betha(1) = alpha(1, 1)*((Kp / 10000.0) * cartXDotDot[2][1].vel[1] + (Kv / 250.0) * cartXDot[2][1].vel[1] + (1.5 * Kp) * cartX[2][1].p[1]);



    //step 1
    jointRates[1](0) = jointRates[0](0) + timeDelta * jointAccelerations[0](0);
    jointRates[1](1) = jointRates[0](1) + timeDelta * jointAccelerations[0](1);
    jointPoses[1](0) = jointPoses[0](0) + timeDelta * jointRates[0](0);
    jointPoses[1](1) = jointPoses[0](1) + timeDelta * jointRates[0](1);

    constraintSolver.CartToJnt(jointPoses[1], jointRates[1], jointAccelerations[1], cartXDotDot[0], alpha, betha, externalNetForce, jointTorques[0]);
    //Actual
    constraintSolver.getLinkCartesianPose(cartX[0]);
    constraintSolver.getLinkCartesianVelocity(cartXDot[0]);
    
    //Error
    cartX[2][1].p[0] = cartX[1][1].p[0] - cartX[0][1].p[0];
    cartX[2][1].p[1] = cartX[1][1].p[1] - cartX[0][1].p[1];
    cartXDot[2][1].vel[0] = cartXDot[1][1].vel[0] - cartXDot[0][1].vel[0];
    cartXDot[2][1].vel[1] = cartXDot[1][1].vel[1] - cartXDot[0][1].vel[1];
    cartXDotDot[2][1].vel[0] = cartXDotDot[1][1].vel[0] - cartXDotDot[0][1].vel[0];
    cartXDotDot[2][1].vel[1] = cartXDotDot[1][1].vel[1] - cartXDotDot[0][1].vel[1];
    printf("Iter 0:   %f      %f     %f         %f        %f      %f\n", cartX[2][1].p[0], cartX[2][1].p[1], cartXDot[2][1].vel[0], cartXDot[2][1].vel[1], cartXDotDot[2][1].vel[1], cartXDotDot[2][1].vel[1]);

    //Regulator or controller
    betha(0) = alpha(0, 0)*((Kp / 10000.0) * cartXDotDot[2][1].vel[0] + (Kv / 200.0) * cartXDot[2][1].vel[0] + (2 * Kp) * cartX[2][1].p[0]);
    betha(1) = alpha(1, 1)*((Kp / 10000.0) * cartXDotDot[2][1].vel[1] + (Kv / 250.0) * cartXDot[2][1].vel[1] + (1.5 * Kp) * cartX[2][1].p[1]);

    //Definition of process main loop
    //-------------------------------------------------------------------------------------//
    for (double t = 2 * timeDelta; t <= TimeConstant; t = t + timeDelta)
    {
        //Interpolation q = a0+a1t+a2t^2+a3t^3
        //Desired
        cartX[1][1].p[0] = b0_x + b1_x * t + b2_x * t * t + b3_x * t * t*t;
        cartX[1][1].p[1] = b0_y + b1_y * t + b2_y * t * t + b3_y * t * t*t;
        cartXDot[1][1].vel[0] = b1_x + 2 * b2_x * t + 3 * b3_x * t*t;
        cartXDot[1][1].vel[1] = b1_y + 2 * b2_y * t + 3 * b3_y * t*t;
        cartXDotDot[1][1].vel[0] = 2 * b2_x + 6 * b3_x*t;
        cartXDotDot[1][1].vel[1] = 2 * b2_y + 6 * b3_y*t;

        // AB 2 order: predictor
        jointRates[2](0) = jointRates[1](0) + timeDelta * (1.5 * jointAccelerations[1](0) - 0.5 * jointAccelerations[0](0));
        jointRates[2](1) = jointRates[1](1) + timeDelta * (1.5 * jointAccelerations[1](1) - 0.5 * jointAccelerations[0](1));
        jointPoses[2](0) = jointPoses[1](0) + timeDelta * (1.5 * jointRates[1](0) - 0.5 * jointRates[0](0));
        jointPoses[2](1) = jointPoses[1](1) + timeDelta * (1.5 * jointRates[1](1) - 0.5 * jointRates[0](1));
        jointTorques[1] = jointTorques[0]; //correction is done on pose,vel and acc, so we need old torques with corrected pose, vel to get corrected acc
        // printf("Prediction: %f          %f %f       %f %f       %f %f           %f %f\n", t, jointPoses[2](0), jointPoses[2](1), jointRates[2](0), jointRates[2](1), jointAccelerations[1](0), jointAccelerations[1](1), jointTorques[0](0), jointTorques[0](1));

        //Function evaluation
        constraintSolver.CartToJnt(jointPoses[2], jointRates[2], jointAccelerations[2], cartXDotDot[0], alpha, betha, externalNetForce, jointTorques[0]);
        //     printf("Evaluation 1: %f          %f %f       %f %f       %f %f           %f %f\n", t, jointPoses[2](0), jointPoses[2](1), jointRates[2](0), jointRates[2](1), jointAccelerations[2](0), jointAccelerations[2](1), jointTorques[0](0), jointTorques[0](1));

        //AM 2 order: corrector
        jointPoses[2](0) = jointPoses[1](0) + timeDelta * ((5 / 12.0) * jointRates[2](0) + (2 / 3.0) * jointRates[1](0) - (1 / 12.0) * jointRates[0](0));
        jointPoses[2](1) = jointPoses[1](1) + timeDelta * ((5 / 12.0) * jointRates[2](1) + (2 / 3.0) * jointRates[1](1) - (1 / 12.0) * jointRates[0](1));
        jointPoses[0] = jointPoses[1];
        jointPoses[1] = jointPoses[2];
        jointRates[2](0) = jointRates[1](0) + timeDelta * ((5 / 12.0) * jointAccelerations[2](0) + (2 / 3.0) * jointAccelerations[1](0) - (1 / 12.0) * jointAccelerations[0](0));
        jointRates[2](1) = jointRates[1](1) + timeDelta * ((5 / 12.0) * jointAccelerations[2](1) + (2 / 3.0) * jointAccelerations[1](1) - (1 / 12.0) * jointAccelerations[0](1));
        jointRates[0] = jointRates[1]; //memorize n+1
        jointRates[1] = jointRates[2]; //memorize n+2
        // printf("Correction : %f          %f %f       %f %f       %f %f           %f %f\n", t, jointPoses[2](0), jointPoses[2](1), jointRates[2](0), jointRates[2](1), jointAccelerations[2](0), jointAccelerations[2](1), jointTorques[0](0), jointTorques[0](1));

        //Function evaluation give final corrected acc n+2
        constraintSolver.CartToJnt(jointPoses[2], jointRates[2], jointAccelerations[2], cartXDotDot[0], alpha, betha, externalNetForce, jointTorques[1]);
        //printf("%f          %f      %f       %f     %f       %f     %f           %f     %f\n", t, jointPoses[2](0), jointPoses[2](1), jointRates[2](0), jointRates[2](1), jointAccelerations[2](0), jointAccelerations[2](1), jointTorques[1](0), jointTorques[1](1));

        jointAccelerations[0] = jointAccelerations[1]; //memorize n+1
        jointAccelerations[1] = jointAccelerations[2]; //memoze n+2
        jointTorques[0] = jointTorques[1];

        constraintSolver.initial_upwards_sweep(jointPoses[1], jointRates[1], jointAccelerations[1], externalNetForce);
        //Actual
        constraintSolver.getLinkCartesianPose(cartX[0]);
        constraintSolver.getLinkCartesianVelocity(cartXDot[0]);

        //Error
        cartX[2][1].p[0] = cartX[1][1].p[0] - cartX[0][1].p[0];
        cartX[2][1].p[1] = cartX[1][1].p[1] - cartX[0][1].p[1];
        cartXDot[2][1].vel[0] = cartXDot[1][1].vel[0] - cartXDot[0][1].vel[0];
        cartXDot[2][1].vel[1] = cartXDot[1][1].vel[1] - cartXDot[0][1].vel[1];
        cartXDotDot[2][1].vel[0] = cartXDotDot[1][1].vel[0] - cartXDotDot[0][1].vel[0];
        cartXDotDot[2][1].vel[1] = cartXDotDot[1][1].vel[1] - cartXDotDot[0][1].vel[1];
        printf("%f          %f      %f     %f         %f        %f      %f\n", t, cartX[2][1].p[0], cartX[2][1].p[1], cartXDot[2][1].vel[0], cartXDot[2][1].vel[1], cartXDotDot[2][1].vel[1], cartXDotDot[2][1].vel[1]);

        //Regulator or controller
        betha(0) = alpha(0, 0)*((Kp / 40000.0) * cartXDotDot[2][1].vel[0] + (Kv / 4000.0) * cartXDot[2][1].vel[0] + (Kp) * cartX[2][1].p[0]);
        betha(1) = alpha(1, 1)*((Kp / 20000.0) * cartXDotDot[2][1].vel[1] + (Kv / 500.0) * cartXDot[2][1].vel[1] + (0.75 * Kp) * cartX[2][1].p[1]);

    }

    return 0;
}
