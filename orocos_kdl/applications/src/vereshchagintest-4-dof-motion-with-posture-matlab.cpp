#include <frames.hpp>
#include <joint.hpp>
#include <frames_io.hpp>
#include <chainidsolver_vereshchagin.hpp>
#include <chainfksolverpos_recursive.hpp>
#include <chain.hpp>
#include <kinfam_io.hpp>
#include <iostream>

//in this experiment both posture and constraint control are present

int main()
{

    using namespace KDL;


    //Definition of kinematic chain
    //-----------------------------------------------------------------------------------------------//
    //Joint (const JointType &type=None, const double &scale=1, const double &offset=0, const double &inertia=0, const double &damping=0, const double &stiffness=0)
    Joint rotJoint0 = Joint(Joint::RotZ, 1, 0, 0.01);
    Joint rotJoint1 = Joint(Joint::RotZ, 1, 0, 0.01);
    Joint rotJoint2 = Joint(Joint::RotZ, 1, 0, 0.01);
    Joint rotJoint3 = Joint(Joint::RotZ, 1, 0, 0.01);

    Frame refFrame(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.0, 0.0, 0.0));
    Frame frame1(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.2, 0.0, 0.0));
    Frame frame2(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.2, 0.0, 0.0));
    Frame frame3(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.2, 0.0, 0.0));
    Frame frame4(Rotation::RPY(0.0, 0.0, 0.0), Vector(0.2, 0.0, 0.0));
    //Segment (const Joint &joint=Joint(Joint::None), const Frame &f_tip=Frame::Identity(), const RigidBodyInertia &I=RigidBodyInertia::Zero())
    Segment segment1 = Segment(rotJoint0, frame1);
    Segment segment2 = Segment(rotJoint1, frame2);
    Segment segment3 = Segment(rotJoint2, frame3);
    Segment segment4 = Segment(rotJoint3, frame4);

    // 	RotationalInertia (double Ixx=0, double Iyy=0, double Izz=0, double Ixy=0, double Ixz=0, double Iyz=0)
    RotationalInertia rotInerSeg1(0.0, 0.0, 0.0, 0.0, 0.0, 0.0); //around symmetry axis of rotation
    //RigidBodyInertia (double m=0, const Vector &oc=Vector::Zero(), const RotationalInertia &Ic=RotationalInertia::Zero())
    RigidBodyInertia inerSegment1(0.5, Vector(0.2, 0.0, 0.0), rotInerSeg1);
    RigidBodyInertia inerSegment2(0.5, Vector(0.2, 0.0, 0.0), rotInerSeg1);
    RigidBodyInertia inerSegment3(0.5, Vector(0.2, 0.0, 0.0), rotInerSeg1);
    RigidBodyInertia inerSegment4(0.5, Vector(0.2, 0.0, 0.0), rotInerSeg1);
    segment1.setInertia(inerSegment1);
    segment2.setInertia(inerSegment2);
    segment3.setInertia(inerSegment3);
    segment4.setInertia(inerSegment4);

    Chain chain;
    chain.addSegment(segment1);
    chain.addSegment(segment2);
    chain.addSegment(segment3);
    chain.addSegment(segment4);
    //~Definition of kinematic chain




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
    Jacobian alpha(2);
    alpha.setColumn(0, constraintForcesX);
    alpha.setColumn(1, constraintForcesY);
    //Acceleration energy at  the end-effector
    JntArray betha(2); //set to zero
    betha(0) = 0.0;
    betha(1) = 0.0;
    //betha(2) = 0.0;
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
    externalNetForce.push_back(externalNetForce1);
    externalNetForce.push_back(externalNetForce2);
    //~Definition of constraints and external disturbances



    //Definition of solver and initial configuration
    //-------------------------------------------------------------------------------------//
    int numberOfConstraints = 2;
    ChainIdSolver_Vereshchagin constraintSolver(chain, twist1, numberOfConstraints);

    //These arrays of joint values contain actual and desired values
    //actual is generated by a solver and integrator
    //desired is given by an interpolator
    //error is the difference between desired-actual
    //0-actual, 1-desired, 2-error
    int k = 4;
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
    //0-actual, 1-desire, 2-error, 3-errorsum
    k = 4;
    Frames cartX[k];
    Twists cartXDot[k];
    Twists cartXDotDot[k];
    Twist accLink;
    for (int i = 0; i < k; i++) //i is number of variables (actual, desired, error)
    {
        for (int j = 0; j < 4; j++) //j is number of links, in this case 4
        {
            cartX[i].push_back(frame1);
            cartXDot[i].push_back(accLink);
            cartXDotDot[i].push_back(accLink);
        }

    }


    // Initial arm position configuration/constraint, negative in clockwise
    JntArray jointFinalPose(chain.getNrOfJoints());
    jointFinalPose(0) = -M_PI/12.0;
    jointFinalPose(1) = M_PI/24.0;
    jointFinalPose(2) = M_PI/4.0;
    jointFinalPose(3) = -M_PI/24.0;
    //correspond to x = 0.723350      y = 0.143883

    JntArray jointInitialPose(chain.getNrOfJoints());
    jointInitialPose(0) = M_PI/4.0;
    jointInitialPose(1) = M_PI/6.0;
    jointInitialPose(2) = -M_PI/8.0;
    jointInitialPose(3) = M_PI/12.0;
    //correspond to x = 0.391474      y = 0.678053 

    //actual initial
    jointPoses[0](0) = jointInitialPose(0);
    jointPoses[0](1) = jointInitialPose(1);
    jointPoses[0](2) = jointInitialPose(2);
    jointPoses[0](3) = jointInitialPose(3);

    //desired initial
    jointPoses[1](0) = jointInitialPose(0);
    jointPoses[1](1) = jointInitialPose(1);
    jointPoses[1](2) = jointInitialPose(2);
    jointPoses[1](3) = jointInitialPose(3);
    //~Definition of solver and initial configuration
    //-------------------------------------------------------------------------------------//

    //Definition of process main loop
    //-------------------------------------------------------------------------------------//
    //-------------------------------------------------------------------------------------//
    double taskTimeConstant = 2.5; //Time required to complete the task move(frameinitialPose, framefinalPose) default T=10.0
    double simulationTime = 2*taskTimeConstant;
    double timeDelta = 0.001;
    double timeToSettle = 0.015;
    int status;
    
    //for cartesian space
    double ksi[2] = {1.0, 1.0}; //damping factor
    double Kp[2];
    Kp[0] = 1.345/(timeToSettle*timeToSettle); // 1.345 for x const only
    Kp[1] = 0.232/(timeToSettle*timeToSettle);
    double Kv[2];
    Kv[0] = 2.5978*ksi[0]/timeToSettle; // 2.5978 for x const only
    Kv[1] = 2.55989*ksi[1]/timeToSettle; // added control around Y using fext improves stability of constraint by Y never follows desired path
    double Ki[2] = {0.0, 0.0};
    double K = 0.005;
   
    
    //For joint space control without constraints using computed torque
    // double ksi[2] = {1.0, 1.0}; //damping factor
    double Kpq[4];
    Kpq[0] = 0.1/(timeToSettle*timeToSettle);
    Kpq[1] = 0.2/(timeToSettle*timeToSettle);
    Kpq[2] = 0.1/(timeToSettle*timeToSettle);
    Kpq[3] = 0.1/(timeToSettle*timeToSettle);
    double Kvq[4];
    Kvq[0] = 2.0*ksi[0]/timeToSettle;
    Kvq[1] = 3.0*ksi[1]/timeToSettle;
    Kvq[2] = 3.0*ksi[1]/timeToSettle;
    Kvq[3] = 3.0*ksi[1]/timeToSettle;
    // double Ki[2] = {1, 1};
    // double Ka[2] = {0.0, 0.0};
    
    //Interpolator parameters:
    //cartesian space
    double b0_y = 0.678053; //should come from initial joint configuration
    double b1_y = 0.0;
    double b2_y =  ((0.143883 - b0_y)*3.0 / (simulationTime * simulationTime)); //xFinal= 0.723350 yFinal = 0.143883 
    double b3_y = -((0.143883 - b0_y)*2.0 / (simulationTime * simulationTime * simulationTime));

    double b0_x = 0.391474; //should come from initial joint configuration
    double b1_x = 0.0;
    double b2_x = ((0.391474 - b0_x)*3.0 / (simulationTime * simulationTime)); //xInit  = 0.391474  Yinit = 0.678053 
    double b3_x = -((0.391474 - b0_x)*2.0 / (simulationTime * simulationTime * simulationTime));
     
    //joint space
    double a0_q0 = jointInitialPose(0);
    double a1_q0 = 0.0;
    double a2_q0 = ((jointFinalPose(0)  - jointInitialPose(0))*3.0 / (simulationTime * simulationTime));
    double a3_q0 = -((jointFinalPose(0) - jointInitialPose(0))*2.0 / (simulationTime * simulationTime * simulationTime));

    double a0_q1 = jointInitialPose(1);
    double a1_q1 = 0.0;
    double a2_q1 = ((jointFinalPose(1) - jointInitialPose(1))*3.0 / (simulationTime * simulationTime));
    double a3_q1 = -((jointFinalPose(1) - jointInitialPose(1))*2.0 / (simulationTime * simulationTime * simulationTime));
    
    double a0_q2 = jointInitialPose(2);
    double a1_q2 = 0.0;
    double a2_q2 = ((jointFinalPose(2) - jointInitialPose(2))*3.0 / (simulationTime * simulationTime));
    double a3_q2 = -((jointFinalPose(2) - jointInitialPose(2))*2.0 / (simulationTime * simulationTime * simulationTime));

    double a0_q3 = jointInitialPose(3);
    double a1_q3 = 0.0;
    double a2_q3 = ((jointFinalPose(3) - jointInitialPose(3))*3.0 / (simulationTime * simulationTime));
    double a3_q3 = -((jointFinalPose(3) - jointInitialPose(3))*2.0 / (simulationTime * simulationTime * simulationTime));

    double feedforwardJointTorque0 = 0.0;
    double feedforwardJointTorque1 = 0.0;
    double feedforwardJointTorque2 = 0.0;
    double feedforwardJointTorque3 = 0.0;
    

    for (double t = 0.0; t <= simulationTime; t = t + timeDelta)
    {
        

        //Interpolation (Desired) q = a0+a1t+a2t^2+a3t^3
        
        cartX[1][3].p[0] = b0_x + b1_x * t + b2_x * t * t + b3_x * t * t*t;
        cartX[1][3].p[1] = b0_y + b1_y * t + b2_y * t * t + b3_y * t * t*t;
        cartXDot[1][3].vel[0] = b1_x + 2 * b2_x * t + 3 * b3_x * t*t;
        cartXDot[1][3].vel[1] = b1_y + 2 * b2_y * t + 3 * b3_y * t*t;
        cartXDotDot[1][3].vel[0] = 2 * b2_x + 6 * b3_x*t;
        cartXDotDot[1][3].vel[1] = 2 * b2_y + 6 * b3_y*t;
        // printf("%f          %f      %f     %f         %f        %f      %f\n", t, cartX[1][3].p[0], cartX[1][3].p[1], cartXDot[1][3].vel[0], cartXDot[1][3].vel[1], cartXDotDot[1][3].vel[0], cartXDotDot[1][3].vel[1]);
        // printf("Desired Cartesian values: %f          %f      %f     %f         %f        %f      %f\n", t, cartX[1][3].p[0], cartX[1][3].p[1], cartXDot[1][3].vel[0], cartXDot[1][3].vel[1], cartXDotDot[1][3].vel[0], cartXDotDot[1][3].vel[1]);
        
        
        jointPoses[1](0) = a0_q0 + a1_q0 * t + a2_q0 * t * t + a3_q0 * t * t*t;
        jointPoses[1](1) = a0_q1 + a1_q1 * t + a2_q1 * t * t + a3_q1 * t * t*t;
        jointPoses[1](2) = a0_q2 + a1_q2 * t + a2_q2 * t * t + a3_q2 * t * t*t;
        jointPoses[1](3) = a0_q3 + a1_q3 * t + a2_q3 * t * t + a3_q3 * t * t*t;

        jointRates[1](0) = a1_q0 + 2 * a2_q0 * t + 3 * a3_q0 * t*t;
        jointRates[1](1) = a1_q1 + 2 * a2_q1 * t + 3 * a3_q1 * t*t;
        jointRates[1](2) = a1_q2 + 2 * a2_q2 * t + 3 * a3_q2 * t*t;
        jointRates[1](3) = a1_q3 + 2 * a2_q3 * t + 3 * a3_q3 * t*t;

        jointAccelerations[1](0) = 2 * a2_q0 + 6 * a3_q0*t;
        jointAccelerations[1](1) = 2 * a2_q1 + 6 * a3_q1*t;
        jointAccelerations[1](2) = 2 * a2_q2 + 6 * a3_q2*t;
        jointAccelerations[1](3) = 2 * a2_q3 + 6 * a3_q3*t;

        // printf("%f\n", jointPoses[1](3));
        //printf("Desired joint values: %f         %f          %f      %f       %f     %f       %f\n", t, jointPoses[1](0), jointPoses[1](1), jointRates[1](0), jointRates[1](1), jointAccelerations[1](0), jointAccelerations[1](1));
        // printf("%f         %f          %f      %f       %f     %f       %f      %f      %f\n", t, jointPoses[1](0), jointPoses[1](1), jointPoses[1](2), jointPoses[1](3), jointRates[1](0), jointRates[1](1), jointRates[1](2), jointRates[1](3));
        

        status = constraintSolver.CartToJnt(jointPoses[0], jointRates[0], jointAccelerations[0],alpha, betha, externalNetForce, jointTorques[0]);  

        constraintSolver.getLinkCartesianPose(cartX[0]);
        constraintSolver.getLinkCartesianVelocity(cartXDot[0]);
        constraintSolver.getLinkCartesianAcceleration(cartXDotDot[0]);
        printf("%f          %f      %f     %f         %f        %f      %f\n", t, cartX[0][3].p.x(), cartX[0][3].p.y(), cartXDot[0][3].vel[0], cartXDot[0][3].vel[1], cartXDotDot[0][3].vel[0], cartXDotDot[0][3].vel[1]);
        // printf("Actual cartesian values: %f          %f      %f     %f         %f        %f      %f\n", t, cartX[0][3].p.x(), cartX[0][3].p.y(), cartXDot[0][3].vel[0], cartXDot[0][3].vel[1], cartXDotDot[0][3].vel[0], cartXDotDot[0][3].vel[1]);

        //Integration(robot joint values for rates and poses; actual) at the given "instanteneous" interval for joint position and velocity.
        jointRates[0](0) = jointRates[0](0) + jointAccelerations[0](0) * timeDelta; //Euler Forward
        jointPoses[0](0) = jointPoses[0](0) + (jointRates[0](0) - jointAccelerations[0](0) * timeDelta / 2.0) * timeDelta; //Trapezoidal rule
        jointRates[0](1) = jointRates[0](1) + jointAccelerations[0](1) * timeDelta; //Euler Forward
        jointPoses[0](1) = jointPoses[0](1) + (jointRates[0](1) - jointAccelerations[0](1) * timeDelta / 2.0) * timeDelta;

        jointRates[0](2) = jointRates[0](2) + jointAccelerations[0](2) * timeDelta; //Euler Forward
        jointPoses[0](2) = jointPoses[0](2) + (jointRates[0](2) - jointAccelerations[0](2) * timeDelta / 2.0) * timeDelta; //Trapezoidal rule
        jointRates[0](3) = jointRates[0](3) + jointAccelerations[0](3) * timeDelta; //Euler Forward
        jointPoses[0](3) = jointPoses[0](3) + (jointRates[0](3) - jointAccelerations[0](3) * timeDelta / 2.0) * timeDelta;
        // printf("%f\n", jointPoses[0](2));
        // printf("%f          %f      %f       %f     %f       %f      %f     %f      %f\n", t, jointPoses[0](0), jointPoses[0](1), jointPoses[0](2), jointPoses[0](3), jointAccelerations[0](0), jointAccelerations[0](1), jointAccelerations[0](2), jointAccelerations[0](3));
        // printf("Actual joint values: %f          %f      %f       %f     %f       %f      %f     %f      %f\n", t, jointPoses[0](0), jointPoses[0](1), jointRates[0](0), jointRates[0](1), jointAccelerations[0](0), jointAccelerations[0](1), jointTorques[0](0), jointTorques[0](1));
        
        //Error
        jointPoses[2](0) = jointPoses[1](0) - jointPoses[0](0);
        jointPoses[2](1) = jointPoses[1](1) - jointPoses[0](1);
        jointPoses[2](2) = jointPoses[1](2) - jointPoses[0](2);
        jointPoses[2](3) = jointPoses[1](3) - jointPoses[0](3);

        jointRates[2](0) = jointRates[1](0) - jointRates[0](0);
        jointRates[2](1) = jointRates[1](1) - jointRates[0](1);
        jointRates[2](2) = jointRates[1](2) - jointRates[0](2);
        jointRates[2](3) = jointRates[1](3) - jointRates[0](3);

        jointAccelerations[2](0) = jointAccelerations[1](0) - jointAccelerations[0](0);
        jointAccelerations[2](1) = jointAccelerations[1](1) - jointAccelerations[0](1);
        jointAccelerations[2](2) = jointAccelerations[1](2) - jointAccelerations[0](2);
        jointAccelerations[2](3) = jointAccelerations[1](3) - jointAccelerations[0](3);

        // jointPoses[3](0) += timeDelta*jointPoses[2](0);
        // jointPoses[3](1) += timeDelta*jointPoses[2](1);
        // printf("%f         %f          %f      %f       %f     %f       %f      %f      %f\n", t, jointPoses[2](0), jointPoses[2](1), jointPoses[2](2), jointPoses[2](3), jointRates[2](0), jointRates[2](1), jointRates[2](2), jointRates[2](3));

        cartX[2][3].p[0] = cartX[1][3].p[0] - cartX[0][3].p[0];
        cartX[2][3].p[1] = cartX[1][3].p[1] - cartX[0][3].p[1];
        cartXDot[2][3].vel[0] = cartXDot[1][3].vel[0] - cartXDot[0][3].vel[0];
        cartXDot[2][3].vel[1] = cartXDot[1][3].vel[1] - cartXDot[0][3].vel[1];
        cartXDotDot[2][3].vel[0] = cartXDotDot[1][3].vel[0] - cartXDotDot[0][3].vel[0];
        cartXDotDot[2][3].vel[1] = cartXDotDot[1][3].vel[1] - cartXDotDot[0][3].vel[1];
        cartX[3][3].p[0] += timeDelta * cartX[2][3].p[0]; //for integral term;
        cartX[3][3].p[1] += timeDelta * cartX[2][3].p[1];
        // printf("%f          %f      %f     %f         %f        %f      %f\n", t, cartX[2][3].p[0], cartX[2][3].p[1], cartXDot[2][3].vel[0], cartXDot[2][3].vel[1], cartXDotDot[2][3].vel[1], cartXDotDot[2][3].vel[1]);

        //Regulator or controller
        //acceleration energy control
        betha(0) = alpha(0, 0)*(K * cartXDotDot[2][3].vel[0] + (Kv[0]) * cartXDot[2][3].vel[0] + (Kp[0]) * cartX[2][3].p[0]+ Ki[1] * cartX[3][3].p[0]); // 0.5xgain
        // betha(1) = alpha(1, 1)*(K * cartXDotDot[2][3].vel[1] + (Kv[0]) * cartXDot[2][3].vel[1] + (Kp[1]) * cartX[2][3].p[1] + Ki[1] * cartX[3][3].p[0]);
        
        feedforwardJointTorque0 = jointAccelerations[1](0) + (Kpq[0])*jointPoses[2](0) + (Kvq[0])*jointRates[2](0);
        // feedforwardJointTorque1 = jointAccelerations[1](1) + (Kpq[1])*jointPoses[2](1) + (Kvq[1])*jointRates[2](1);//computed joint torque control
        // feedforwardJointTorque2 = jointAccelerations[1](2) + (Kpq[2])*jointPoses[2](2) + (Kvq[2])*jointRates[2](2);
        // feedforwardJointTorque3 = jointAccelerations[1](3) + (Kpq[3])*jointPoses[2](3) + (Kvq[3])*jointRates[2](3);
        
        jointTorques[0](0) = jointTorques[0](0) + feedforwardJointTorque0;
        // jointTorques[0](1) = jointTorques[0](1) + feedforwardJointTorque1;
        // jointTorques[0](2) = jointTorques[0](2) + feedforwardJointTorque2;
        // jointTorques[0](3) = jointTorques[0](3) + feedforwardJointTorque3;
        /*
        //For cartesian space control one needs to calculate from the obtained joint space value, new cartesian space poses.
        //Then based on the difference of the signal (desired-actual) we define a regulation function (controller)
        // this difference should be compensated either by joint torques.
        externalNetForce[3].force[0] = cartXDotDot[2][3].vel[0] + ((Kv[0]) * cartXDot[2][3].vel[0] + (Kp[0]) * cartX[2][3].p[0] + (Ki[0]) * cartX[3][3].p[0]);
        externalNetForce[3].force[1] = cartXDotDot[2][3].vel[1] + ((Kv[1]) * cartXDot[2][3].vel[1] + (Kp[1]) * cartX[2][3].p[1] + (Ki[1]) * cartX[3][3].p[1]);
        */
        // externalNetForce[3].force[0] = ((Kv[0]) * cartXDot[2][3].vel[0] + (Kp[0]) * cartX[2][3].p[0] );
        externalNetForce[3].force[1] = ((Kv[1]) * cartXDot[2][3].vel[1] + (Kp[1]) * cartX[2][3].p[1] ); 

    }
    //~Definition of process main loop
    //-------------------------------------------------------------------------------------//





    return 0;
}
