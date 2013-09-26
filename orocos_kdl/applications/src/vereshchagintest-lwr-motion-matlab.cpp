

#include <frames.hpp>
#include <joint.hpp>
#include <frames_io.hpp>
#include <chainidsolver_vereshchagin.hpp>
#include <chainfksolverpos_recursive.hpp>
#include <chain.hpp>
#include <kinfam_io.hpp>
#include <iostream>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

// #define DESIRED_CARTESIAN_VALUES
// #define DESIRED_JOINT_VALUES
#define ACTUAL_CARTESIAN_VALUES
// #define ACTUAL_JOINT_VALUES
// #define CARTESIAN_ERROR_VALUES
// #define JOINT_ERROR_VALUES
// #define FKPOSE_TEST
// #define X_CONSTRAINT_SET
#define Y_CONSTRAINT_SET
// #define Z_CONSTRAINT_SET

int main()
{

    using namespace KDL;
   
    //*******************************************************************
    // LWR ROBOT MODEL PARAMETERS (DO NOT DISTRIBUTE OUTSIDE PMA)
    //*******************************************************************

    Chain chainLWR;
    //joint 0
    // chainLWR.addSegment(Segment(Joint(Joint::None),
    //               Frame::DH_Craig1989(0.0, 0.0, 0.31, 0.0)
    //               ));
    //joint 1
    chainLWR.addSegment(Segment(Joint(Joint::RotZ),
                  Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0),
                  Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0).Inverse()*RigidBodyInertia(2,
                                                 KDL::Vector::Zero(),
                                                 RotationalInertia(0.0,0.0,0.0115343,0.0,0.0,0.0))));
                   
    //joint 2 
    chainLWR.addSegment(Segment(Joint(Joint::RotZ),
                  Frame::DH_Craig1989(0.0, -1.5707963, 0.4, 0.0),
                  Frame::DH_Craig1989(0.0, -1.5707963, 0.4, 0.0).Inverse()*RigidBodyInertia(2,
                                                   KDL::Vector(0.0,-0.3120511,-0.0038871),
                                                   RotationalInertia(-0.5471572,-0.0000302,-0.5423253,0.0,0.0,0.0018828))));
                  
    //joint 3
    chainLWR.addSegment(Segment(Joint(Joint::RotZ),
                  Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0),
                  Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0).Inverse()*RigidBodyInertia(2,
                                                   KDL::Vector(0.0,-0.0015515,0.0),
                                                   RotationalInertia(0.0063507,0.0,0.0107804,0.0,0.0,-0.0005147))));
                  
    //joint 4
    chainLWR.addSegment(Segment(Joint(Joint::RotZ),
                  Frame::DH_Craig1989(0.0, 1.5707963, 0.39, 0.0),
                  Frame::DH_Craig1989(0.0, 1.5707963, 0.39, 0.0).Inverse()*RigidBodyInertia(2,
                                                   KDL::Vector(0.0,0.5216809,0.0),
                                                   RotationalInertia(-1.0436952,0.0,-1.0392780,0.0,0.0,0.0005324))));
                  
    //joint 5
    chainLWR.addSegment(Segment(Joint(Joint::RotZ),
                  Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0),
                  Frame::DH_Craig1989(0.0, 1.5707963, 0.0, 0.0).Inverse()*RigidBodyInertia(2,
                                                   KDL::Vector(0.0,0.0119891,0.0),
                                                   RotationalInertia(0.0036654,0.0,0.0060429,0.0,0.0,0.0004226))));
                  
    //joint 6
    chainLWR.addSegment(Segment(Joint(Joint::RotZ),
                  Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0),
                  Frame::DH_Craig1989(0.0, -1.5707963, 0.0, 0.0).Inverse()*RigidBodyInertia(2,
                                                   KDL::Vector(0.0,0.0080787,0.0),
                                                   RotationalInertia(0.0010431,0.0,0.0036376,0.0,0.0,0.0000101))));
    //joint 7
    chainLWR.addSegment(Segment(Joint(Joint::RotZ),
                   Frame::Identity(),
                   RigidBodyInertia(2,
                                                   KDL::Vector::Zero(),
                                                   RotationalInertia(0.000001,0.0,0.0001203,0.0,0.0,0.0))));


    //Definition of constraints and external disturbances
    //--------------------------------------------------------------------------------------//
    //Constraint force matrix at the end-effector
    //What is the convention for the spatial force matrix; is it the same as in thesis?
    //NOTE AZAMAT: Constraint are also defined in local reference frame?!
    unsigned int numberOfJoints = chainLWR.getNrOfJoints();
    unsigned int numberOfLinks = chainLWR.getNrOfSegments();
    // std::cout << "number of chain joints " << numberOfJoints << std::endl;;
    // std::cout << "number of chain links " << numberOfLinks << std::endl;

    unsigned int numberOfConstraints = 1;
    Twist constraintForce;
    Twists constraintForces;
    for (unsigned int i = 0; i < numberOfJoints; i++)
    {   
        SetToZero(constraintForce);
        constraintForces.push_back(constraintForce);
    }

    constraintForces[0].vel[0] = 0; //Xlinear
    constraintForces[1].vel[1] = 0; //Ylinear
    constraintForces[2].vel[2] = 0; //Zlinear

    #ifdef X_CONSTRAINT_SET
        constraintForces[0].vel[0] = 1;
    #endif //~X_CONSTRAINT_SET

    #ifdef Y_CONSTRAINT_SET
        constraintForces[1].vel[1] = 1;
    #endif //~Y_CONSTRAINT_SET

    #ifdef Z_CONSTRAINT_SET
        constraintForces[2].vel[2] = 1;
    #endif //~Z_CONSTRAINT_SET
    //--------------------------------------------------------------------------------------//
    //Acceleration energy and constraint matrix at  the end-effector
    Jacobian alpha(numberOfConstraints);
    JntArray bethaControl(numberOfConstraints); //set to zero
    JntArray betha(numberOfConstraints); //set to zero
    for (unsigned int i = 0; i < numberOfConstraints; i++)
    {
        alpha.setColumn(i, constraintForces[i]);
        betha(i) = 0.0;
        bethaControl(i) = 0.0;

    }

    //arm root acceleration
    Vector linearAcc(0.0, 0.0, 9.8); //gravitational acceleration along Z
    Vector angularAcc(0.0, 0.0, 0.0);
    Twist twist0(linearAcc, angularAcc);

    //external forces on the arm
    Wrench externalNetForce;
    Wrenches externalNetForces;
    for (unsigned int i = 0; i < numberOfLinks; i++)
    {
        externalNetForces.push_back(externalNetForce);
    }
    //~Definition of constraints and external disturbances


    //Definition of solver and initial configuration

    ChainIdSolver_Vereshchagin constraintSolver(chainLWR, twist0, numberOfConstraints);
   
    //These arrays of joint values contain actual and desired values
    //actual is generated by a solver and integrator
    //desired is given by an interpolator
    //error is the difference between desired-actual
    //0-actual, 1-desired, 2-error, 3-sum of error

    unsigned int k = 4;
    JntArray jointPoses[k];
    JntArray jointRates[k];
    JntArray jointAccelerations[k];
    JntArray jointTorques[k];
    JntArray jointControlTorques[k];
    for (unsigned int i = 0; i < k; i++)
    {
        JntArray jointValues(numberOfJoints);
        jointPoses[i] = jointValues;
        jointRates[i] = jointValues;
        jointAccelerations[i] = jointValues;
        jointTorques[i] = jointValues;
        jointControlTorques[i] = jointValues;
    }

    //cartesian space/link values
    //0-actual, 1-desire, 2-error, 3-errorsum
    Frames cartX[k];
    Twists cartXDot[k];
    Twists cartXDotDot[k];
    Twist accLink;
    KDL::Frame frameTemp;
    for (unsigned int i = 0; i < k; i++) //i is number of variables (actual, desired, error)
    {
        for (unsigned int j = 0; j < numberOfLinks; j++) //j is number of links
        {
            cartX[i].push_back(frameTemp);
            cartXDot[i].push_back(accLink);
            cartXDotDot[i].push_back(accLink);
        }

    }


    // Initial arm position configuration/constraint, negative in clockwise
    JntArray jointInitialPose(numberOfJoints);
    jointInitialPose(0) = 0.0;
    jointInitialPose(1) = M_PI/3.0;
    jointInitialPose(2) = M_PI/12.0;
    jointInitialPose(3) = M_PI/6.0;
    jointInitialPose(4) = M_PI/16.0;
    jointInitialPose(5) = -M_PI/12.0;
    jointInitialPose(6) = 0.0;

     // -0.544732      0.050470     0.531996
    JntArray jointFinalPose(numberOfJoints);
    jointFinalPose(0) = M_PI/6.0;
    jointFinalPose(1) = M_PI/4.0;
    jointFinalPose(2) = M_PI/16.0;
    jointFinalPose(3) = M_PI/24.0;
    jointFinalPose(4) = 0.0;
    jointFinalPose(5) = M_PI/12.0;
    jointFinalPose(6) = -M_PI/24.0;
    // corresponds to -0.456123,   -0.251875,    0.591559
    for (unsigned int i = 0; i < numberOfJoints; i++)
    {
        //actual initial
        jointPoses[0](i) = jointInitialPose(i);
        //desired initial
        jointPoses[1](i) = jointInitialPose(i);
    }

    //test
    #ifdef FKPOSE_TEST
        ChainFkSolverPos_recursive fksolver(chainLWR);
        Frame tempEE;
        fksolver.JntToCart(jointPoses[0], tempEE, 7);
        std::cout << tempEE << std::endl;
        fksolver.JntToCart(jointFinalPose, tempEE, 7);
        std::cout << tempEE << std::endl;
    #endif//~FKPOSE_TEST

    //~Definition of solver and initial configuration
    //-------------------------------------------------------------------------------------//


    //Definition of process main loop
    //-------------------------------------------------------------------------------------//
    double taskTimeConstant = 2.5; //Time required to complete the task move(frameinitialPose, framefinalPose) default T=10.0
    double simulationTime = 4*taskTimeConstant;
    double timeDelta = 0.001;
    double timeToSettle = 0.5; //change 
    int status;
    
    //for cartesian space
    double ksi[2] = {1.0, 1.0}; //damping factor
    double Kp[3];
    Kp[0] = 170.015345/(timeToSettle*timeToSettle); // 1.345 for x const only  with timeToSettle= 0.015 and q1 is controlled
    Kp[1] = 170.0232/(timeToSettle*timeToSettle);
    Kp[2] = 170.232/(timeToSettle*timeToSettle);
    double Kv[3];
    Kv[0] = 30.0205978*ksi[0]/timeToSettle; // 2.5978 for x const only with timeToSettle= 0.015 and q1 is controlled
    Kv[1] = 30.055989*ksi[1]/timeToSettle; // added control around Y using fext improves stability of constraint by Y never follows desired path
    Kv[2] = 30.55989*ksi[1]/timeToSettle; 
    double Ki[2] = {0.0, 0.0};
    double K = 0.01;
  
    //For joint space control without constraints using computed torque
    // double ksi[2] = {1.0, 1.0}; //damping factor
    double Kpq[numberOfJoints];
    Kpq[0] = 100.5/(timeToSettle*timeToSettle); 
    Kpq[1] = 100.2/(timeToSettle*timeToSettle);
    Kpq[2] = 75.1/(timeToSettle*timeToSettle);
    Kpq[3] = 40.1/(timeToSettle*timeToSettle);
    Kpq[4] = 40.1/(timeToSettle*timeToSettle);
    Kpq[5] = 40.1/(timeToSettle*timeToSettle);
    Kpq[6] = 10.1/(timeToSettle*timeToSettle);
    double Kvq[numberOfJoints];
    Kvq[0] = 2.0*ksi[0]/timeToSettle;   
    Kvq[1] = 10.0*ksi[1]/timeToSettle;
    Kvq[2] = 3.0*ksi[1]/timeToSettle;
    Kvq[3] = 3.0*ksi[1]/timeToSettle;
    Kvq[4] = 3.0*ksi[1]/timeToSettle;
    Kvq[5] = 3.0*ksi[1]/timeToSettle;
    Kvq[6] = 0.03*ksi[1]/timeToSettle;

    //Interpolator parameters:
    // initial -0.544732      0.050470     0.531996
    // final  -0.456123,   -0.251875,    0.591559
    //cartesian space
    double b0_x = -0.544732; //should come from initial joint configuration
    double b1_x = 0.0;
    double b2_x = ((-0.456123 - b0_x)*3.0 / (simulationTime * simulationTime)); 
    double b3_x = -((-0.456123 - b0_x)*2.0 / (simulationTime * simulationTime * simulationTime));

    double b0_y = 0.050470; //should come from initial joint configuration
    double b1_y = 0.0;
    double b2_y =  ((0.050470  - b0_y)*3.0 / (simulationTime * simulationTime)); 
    double b3_y = -((0.050470  - b0_y)*2.0 / (simulationTime * simulationTime * simulationTime));

    double b0_z = 0.531996; //should come from initial joint configuration
    double b1_z = 0.0;
    double b2_z = ((0.591559 - b0_z)*3.0 / (simulationTime * simulationTime)); 
    double b3_z = -((0.591559 - b0_z)*2.0 / (simulationTime * simulationTime * simulationTime));

     
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


    double a0_q4 = jointInitialPose(4);
    double a1_q4 = 0.0;
    double a2_q4 = ((jointFinalPose(4) - jointInitialPose(4))*3.0 / (simulationTime * simulationTime));
    double a3_q4 = -((jointFinalPose(4) - jointInitialPose(4))*2.0 / (simulationTime * simulationTime * simulationTime));
    
    double a0_q5 = jointInitialPose(5);
    double a1_q5 = 0.0;
    double a2_q5 = ((jointFinalPose(5) - jointInitialPose(5))*3.0 / (simulationTime * simulationTime));
    double a3_q5 = -((jointFinalPose(5) - jointInitialPose(5))*2.0 / (simulationTime * simulationTime * simulationTime));

    double a0_q6 = jointInitialPose(6);
    double a1_q6 = 0.0;
    double a2_q6 = ((jointFinalPose(6) - jointInitialPose(6))*3.0 / (simulationTime * simulationTime));
    double a3_q6 = -((jointFinalPose(6) - jointInitialPose(6))*2.0 / (simulationTime * simulationTime * simulationTime));


    double feedforwardJointTorque0 = 0.0;
    double feedforwardJointTorque1 = 0.0;
    double feedforwardJointTorque2 = 0.0;
    double feedforwardJointTorque3 = 0.0;
    double feedforwardJointTorque4 = 0.0;
    double feedforwardJointTorque5 = 0.0;
    double feedforwardJointTorque6 = 0.0;
    //---------------------------------------------------------------------------//

    //Main simulation loop
    for (double t = 0.0; t <= simulationTime; t = t + timeDelta)
    {
        
        //Interpolation (Desired) q = a0+a1t+a2t^2+a3t^3
    
        cartX[1][numberOfLinks-1].p[0] = b0_x + b1_x * t + b2_x * t * t + b3_x * t * t*t;
        cartX[1][numberOfLinks-1].p[1] = b0_y + b1_y * t + b2_y * t * t + b3_y * t * t*t;
        cartX[1][numberOfLinks-1].p[2] = b0_z + b1_z * t + b2_z * t * t + b3_z * t * t*t;
        cartXDot[1][numberOfLinks-1].vel[0] = b1_x + 2 * b2_x * t + 3 * b3_x * t*t;
        cartXDot[1][numberOfLinks-1].vel[1] = b1_y + 2 * b2_y * t + 3 * b3_y * t*t;
        cartXDot[1][numberOfLinks-1].vel[2] = b1_z + 2 * b2_z * t + 3 * b3_z * t*t;
        cartXDotDot[1][numberOfLinks-1].vel[0] = 2 * b2_x + 6 * b3_x*t;
        cartXDotDot[1][numberOfLinks-1].vel[1] = 2 * b2_y + 6 * b3_y*t;
        cartXDotDot[1][numberOfLinks-1].vel[2] = 2 * b2_z + 6 * b3_z*t;
        
        #ifdef DESIRED_CARTESIAN_VALUES
            printf("%f          %f      %f     %f         %f        %f      %f\n", t, cartX[1][numberOfLinks-1].p[0], cartX[1][numberOfLinks-1].p[1], cartX[1][numberOfLinks-1].p[2], cartXDot[1][numberOfLinks-1].vel[0], cartXDot[1][numberOfLinks-1].vel[1], cartXDot[1][numberOfLinks-1].vel[2]);
            // printf("Desired Cartesian values: %f          %f      %f     %f         %f        %f      %f\n", t, cartX[1][numberOfLinks-1].p[0], cartX[1][numberOfLinks-1].p[1], cartX[1][numberOfLinks-1].p[2], cartXDot[1][numberOfLinks-1].vel[0], cartXDot[1][numberOfLinks-1].vel[1], cartXDot[1][numberOfLinks-1].vel[2]);
        #endif //~DESIRED_CARTESIAN_VALUES
        
      
       
        jointPoses[1](0) = a0_q0 + a1_q0 * t + a2_q0 * t * t + a3_q0 * t * t*t;
        jointPoses[1](1) = a0_q1 + a1_q1 * t + a2_q1 * t * t + a3_q1 * t * t*t;
        jointPoses[1](2) = a0_q2 + a1_q2 * t + a2_q2 * t * t + a3_q2 * t * t*t;
        jointPoses[1](3) = a0_q3 + a1_q3 * t + a2_q3 * t * t + a3_q3 * t * t*t;
        jointPoses[1](4) = a0_q4 + a1_q4 * t + a2_q4 * t * t + a3_q4 * t * t*t;  
        jointPoses[1](5) = a0_q5 + a1_q5 * t + a2_q5 * t * t + a3_q5 * t * t*t;
        jointPoses[1](6) = a0_q6 + a1_q6 * t + a2_q6 * t * t + a3_q6 * t * t*t;
        
        jointRates[1](0) = a1_q0 + 2 * a2_q0 * t + 3 * a3_q0 * t*t;
        jointRates[1](1) = a1_q1 + 2 * a2_q1 * t + 3 * a3_q1 * t*t;
        jointRates[1](2) = a1_q2 + 2 * a2_q2 * t + 3 * a3_q2 * t*t;
        jointRates[1](3) = a1_q3 + 2 * a2_q3 * t + 3 * a3_q3 * t*t;
        jointRates[1](4) = a1_q4 + 2 * a2_q4 * t + 3 * a3_q4 * t*t;
        jointRates[1](5) = a1_q5 + 2 * a2_q5 * t + 3 * a3_q5 * t*t;
        jointRates[1](6) = a1_q6 + 2 * a2_q6 * t + 3 * a3_q6 * t*t;

        jointAccelerations[1](0) = 2 * a2_q0 + 6 * a3_q0*t;
        jointAccelerations[1](1) = 2 * a2_q1 + 6 * a3_q1*t;
        jointAccelerations[1](2) = 2 * a2_q2 + 6 * a3_q2*t;
        jointAccelerations[1](3) = 2 * a2_q3 + 6 * a3_q3*t;
        jointAccelerations[1](4) = 2 * a2_q4 + 6 * a3_q4*t;
        jointAccelerations[1](5) = 2 * a2_q5 + 6 * a3_q5*t;
        jointAccelerations[1](6) = 2 * a2_q6 + 6 * a3_q6*t;

        #ifdef DESIRED_JOINT_VALUES
            printf("%f       %f     %f      %f     %f    %f     %f     %f\n", t,jointPoses[1](0), jointPoses[1](1),jointPoses[1](2), jointPoses[1](3), jointPoses[1](4), jointPoses[1](5),jointPoses[1](6));
        // printf("%f       %f     %f      %f     %f    %f     %f     %f      %f     %f    %f\n", t,jointRates[1](0), jointRates[1](1), jointRates[1](2), jointRates[1](3), jointRates[1](4), jointRates[1](5), jointRates[1](6));
        #endif //~DESIRED_JOINT_VALUES

        status = constraintSolver.CartToJnt(jointPoses[0], jointRates[0], jointAccelerations[0], alpha, betha, bethaControl, externalNetForces, jointTorques[0], jointControlTorques[0]);
        constraintSolver.getLinkCartesianPose(cartX[0]);
        constraintSolver.getLinkCartesianVelocity(cartXDot[0]);
        constraintSolver.getLinkCartesianAcceleration(cartXDotDot[0]);
        double alfa(0.0), gamma(0.0), beta(0.0);
        cartX[0][numberOfLinks-1].M.GetEulerZYX(alfa,gamma,beta);
        
        #ifdef ACTUAL_CARTESIAN_VALUES
            printf("%f          %f      %f     %f         %f        %f      %f\n",  t, cartX[0][numberOfLinks-1].p[0], cartX[0][numberOfLinks-1].p[1], cartX[0][numberOfLinks-1].p[2], alfa, gamma, beta);
        #endif //~ACTUAL_CARTESIAN_VALUES
        
        //Integration(robot joint values for rates and poses; actual) at the given "instanteneous" interval for joint position and velocity.
        for (unsigned int i = 0; i < numberOfJoints; i++)
        {
            jointRates[0](i) = jointRates[0](i) + jointAccelerations[0](i) * timeDelta; //Euler Forward
            jointPoses[0](i) = jointPoses[0](i) + (jointRates[0](i) - jointAccelerations[0](i) * timeDelta / 2.0) * timeDelta; //Trapezoidal rule
        }

        #ifdef ACTUAL_JOINT_VALUES
            printf("%f       %f     %f      %f     %f    %f     %f     %f      %f     %f    %f    %f       %f     %f      %f     %f    %f     %f     %f      %f     %f    %f    %f       %f     %f      %f     %f    %f     %f\n", 
            t,jointPoses[0](0), jointPoses[0](1),jointPoses[0](2), jointPoses[0](3), jointPoses[0](4), jointPoses[0](5),jointPoses[0](6), 
            jointRates[0](0), jointRates[0](1), jointRates[0](2), jointRates[0](3), jointRates[0](4), jointRates[0](5), jointRates[0](6),
            jointAccelerations[0](0), jointAccelerations[0](1), jointAccelerations[0](2), jointAccelerations[0](3), jointAccelerations[0](4), jointAccelerations[0](5), jointAccelerations[0](6),
            jointTorques[0](0), jointTorques[0](1), jointTorques[0](2), jointTorques[0](3), jointTorques[0](4), jointTorques[0](5), jointTorques[0](6));
        #endif //~ACTUAL_JOINT_VALUES

        //Error
        for (unsigned int i = 0; i < numberOfJoints; i++)
        {
            jointPoses[2](i) = jointPoses[1](i) - jointPoses[0](i);
            jointRates[2](i) = jointRates[1](i) - jointRates[0](i);
            jointAccelerations[2](i) = jointAccelerations[1](i) - jointAccelerations[0](i);            
        }        
        
        #ifdef JOINT_ERROR_VALUES        
            printf("%f       %f     %f      %f     %f    %f     %f     %f\n", t,jointPoses[2](0), jointPoses[2](1),jointPoses[2](2), jointPoses[2](3), jointPoses[2](4), jointPoses[2](5),jointPoses[2](6));
        #endif //~JOINT_ERROR_VALUES

        cartX[2][numberOfLinks-1].p[0] = cartX[1][numberOfLinks-1].p[0] - cartX[0][numberOfLinks-1].p[0];
        cartX[2][numberOfLinks-1].p[1] = cartX[1][numberOfLinks-1].p[1] - cartX[0][numberOfLinks-1].p[1];
        cartX[2][numberOfLinks-1].p[2] = cartX[1][numberOfLinks-1].p[2] - cartX[0][numberOfLinks-1].p[2];
        cartXDot[2][numberOfLinks-1].vel[0] = cartXDot[1][numberOfLinks-1].vel[0] - cartXDot[0][numberOfLinks-1].vel[0];
        cartXDot[2][numberOfLinks-1].vel[1] = cartXDot[1][numberOfLinks-1].vel[1] - cartXDot[0][numberOfLinks-1].vel[1];
        cartXDot[2][numberOfLinks-1].vel[2] = cartXDot[1][numberOfLinks-1].vel[2] - cartXDot[0][numberOfLinks-1].vel[2];
        cartXDotDot[2][numberOfLinks-1].vel[0] = cartXDotDot[1][numberOfLinks-1].vel[0] - cartXDotDot[0][numberOfLinks-1].vel[0];
        cartXDotDot[2][numberOfLinks-1].vel[1] = cartXDotDot[1][numberOfLinks-1].vel[1] - cartXDotDot[0][numberOfLinks-1].vel[1];
        cartXDotDot[2][numberOfLinks-1].vel[2] = cartXDotDot[1][numberOfLinks-1].vel[2] - cartXDotDot[0][numberOfLinks-1].vel[2];
        cartX[3][numberOfLinks-1].p[0] += timeDelta * cartX[2][numberOfLinks-1].p[0]; //for integral term;
        cartX[3][numberOfLinks-1].p[1] += timeDelta * cartX[2][numberOfLinks-1].p[1];
        cartX[3][numberOfLinks-1].p[2] += timeDelta * cartX[2][numberOfLinks-1].p[2];
        
        #ifdef CARTESIAN_ERROR_VALUES
            printf("%f          %f      %f     %f         %f        %f      %f\n", t, cartX[2][numberOfLinks-1].p[0], cartX[2][numberOfLinks-1].p[1], cartX[2][numberOfLinks-1].p[2], cartXDot[2][numberOfLinks-1].vel[0], cartXDot[2][numberOfLinks-1].vel[1], cartXDot[2][numberOfLinks-1].vel[2]);
        #endif //~CARTESIAN_ERROR_VALUES

        //Regulator or controller
        //acceleration energy control
        betha(0) = alpha(0, 0)*(K * cartXDotDot[2][numberOfLinks-1].vel[0] + (Kv[0]) * cartXDot[2][numberOfLinks-1].vel[0] + (Kp[0]) * cartX[2][numberOfLinks-1].p[0]+ Ki[1] * cartX[3][numberOfLinks-1].p[0]); 
        // bethaControl(1) = alpha(1, 1)*(K * cartXDotDot[2][numberOfLinks-1].vel[1] + (Kv[1]) * cartXDot[2][numberOfLinks-1].vel[1] + (Kp[1]) * cartX[2][numberOfLinks-1].p[1] + Ki[1] * cartX[3][numberOfLinks-1].p[1]);
        // bethaControl(2) = alpha(2, 2)*(K * cartXDotDot[2][numberOfLinks-1].vel[2] + (Kv[2]) * cartXDot[2][numberOfLinks-1].vel[2] + (Kp[2]) * cartX[2][numberOfLinks-1].p[2]+ Ki[1] * cartX[3][numberOfLinks-1].p[2]); 
        
        // priority posture control
        // jointControlTorques[0](0) = jointAccelerations[1](0) + (Kpq[0])*jointPoses[2](0) + (Kvq[0])*jointRates[2](0);
        // jointControlTorques[0](1) = jointAccelerations[1](1) + (Kpq[1])*jointPoses[2](1) + (Kvq[1])*jointRates[2](1);//computed joint torque control
        // jointControlTorques[0](2) = jointAccelerations[1](2) + (Kpq[2])*jointPoses[2](2) + (Kvq[2])*jointRates[2](2);
        // jointControlTorques[0](3) = jointAccelerations[1](3) + (Kpq[3])*jointPoses[2](3) + (Kvq[3])*jointRates[2](3);
        
        //priority constraint control
        // feedforwardJointTorque0 = jointAccelerations[1](0) + (Kpq[0])*jointPoses[2](0) + (Kvq[0])*jointRates[2](0);
        // feedforwardJointTorque1 = jointAccelerations[1](1) + (Kpq[1])*jointPoses[2](1) + (Kvq[1])*jointRates[2](1);//computed joint torque control
        // feedforwardJointTorque2 = jointAccelerations[1](2) + (Kpq[2])*jointPoses[2](2) + (Kvq[2])*jointRates[2](2);
        // feedforwardJointTorque3 = jointAccelerations[1](3) + (Kpq[3])*jointPoses[2](3) + (Kvq[3])*jointRates[2](3);
        // feedforwardJointTorque4 = jointAccelerations[1](4) + (Kpq[4])*jointPoses[2](4) + (Kvq[4])*jointRates[2](4);//computed joint torque control
        // feedforwardJointTorque5 = jointAccelerations[1](5) + (Kpq[5])*jointPoses[2](5) + (Kvq[5])*jointRates[2](5);
        // feedforwardJointTorque6 = jointAccelerations[1](6) + (Kpq[6])*jointPoses[2](6) + (Kvq[6])*jointRates[2](6);
        // jointTorques[0](0) = jointTorques[0](0) + feedforwardJointTorque0;
        // jointTorques[0](1) = jointTorques[0](1) + feedforwardJointTorque1;
        // jointTorques[0](2) = jointTorques[0](2) + feedforwardJointTorque2;
        // jointTorques[0](3) = jointTorques[0](3) + feedforwardJointTorque3;
        // jointTorques[0](4) = jointTorques[0](4) + feedforwardJointTorque4;
        // jointTorques[0](5) = jointTorques[0](5) + feedforwardJointTorque5;
        // jointTorques[0](6) = jointTorques[0](6) + feedforwardJointTorque6;
        
        //For cartesian space control one needs to calculate from the obtained joint space value, new cartesian space poses.
        //Then based on the difference of the signal (desired-actual) we define a regulation function (controller)
        // this difference should be compensated either by joint torques.
        externalNetForces[numberOfLinks-1].force[0] = ((Kv[0]) * cartXDot[2][numberOfLinks-1].vel[0] + (Kp[0]) * cartX[2][numberOfLinks-1].p[0] );
        // externalNetForces[numberOfLinks-1].force[1] = ((Kv[1]) * cartXDot[2][numberOfLinks-1].vel[1] + (Kp[1]) * cartX[2][numberOfLinks-1].p[1] ); 
        externalNetForces[numberOfLinks-1].force[2] = ((Kv[2]) * cartXDot[2][numberOfLinks-1].vel[2] + (Kp[2]) * cartX[2][numberOfLinks-1].p[2] ); 


    }
    //~Definition of process main loop
    //-------------------------------------------------------------------------------------//





    return 0;
}

