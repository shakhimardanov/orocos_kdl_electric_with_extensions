// Copyright  (C)  2009

// Version: 1.0
// Author:
// Maintainer:
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include "treeidsolver_vereshchagin.hpp"
#include "frames_io.hpp"
#include "utilities/svd_eigen_HH.hpp"


namespace KDL
{
using namespace Eigen;

TreeIdSolver_Vereshchagin::TreeIdSolver_Vereshchagin(const Tree& tree_in, const std::string& link, Twist root_acc, unsigned int _nc)
{
    tree = tree_in;
    nj = tree_in.getNrOfJoints();
    ns = tree_in.getNrOfSegments();
    nc = _nc;
    segmentResults = std::vector<segment_info > (ns + 1, segment_info(nc)); // this is fine as a vector because we don't directly work with it by modify it through an external reference.
    acc_root = root_acc;
    constraintLinkName = link;

    //Provide the necessary memory for computing the inverse of M0
    nu_sum.resize(nc);
    M_0_inverse.resize(nc, nc);
    Um = MatrixXd::Identity(nc, nc);
    Vm = MatrixXd::Identity(nc, nc);
    Sm = VectorXd::Ones(nc);
    tmpm = VectorXd::Ones(nc);

    if (serializeTreeToChain() != 0)
    {
        std::cout << "Error: unable to serialize a tree" << std::endl;
        exit(1);
    }

}

int TreeIdSolver_Vereshchagin::serializeTreeToChain()
{
    //this whole serialization of the tree into chains should go into other configuration step methods and not appear in computation.
    //for the time being it is done here for the sake of functionality testing

    SegmentMap::const_iterator iter;
    SegmentMap::const_iterator iterRoot = tree.getRootSegment(); //get root segment iterator(pointer)
    for (iter = tree.getSegments().begin(); iter != tree.getSegments().end(); iter++)
    {

        //Find branching point
        if (iter->second.children.size() > 1) // branching link has more than 1 children
        {
            branchingLinkName.push_back(iter->first);
            //std::cout << "Branching link name " << iter->first << std::endl;
        }
        //find end effector segments
        if (iter->second.children.size() == 0) //ee link does not have any children
        {
            eeLinkName.push_back(iter->first);
            //std::cout << "End effector name " << iter->first << std::endl;

        }
    }
    //create chains from root to ee_s
    for (std::vector<std::string>::iterator iterEE = eeLinkName.begin(); iterEE != eeLinkName.end(); iterEE++)
    {
        Chain currentChain;
        tree.getChain(iterRoot->first, *iterEE, currentChain);
        /* if(iter->first == constraintLinkName)
         {
             if(iter->second.children.size() == 1)
             {

             }
         }
         */
        chainInTree.push_back(currentChain);

    }

    //find links which appear more than once in generated serial chains

    return 0;
}




//int TreeIdSolver_Vereshchagin::CartToJnt(const JntArray& q, const JntArray& q_dot, JntArray& q_dotdot, const std::vector<Jacobian>& alfas, const std::vector<JntArray>& betas, const std::vector<std::string>& link, const Wrenches& f_ext, JntArray &torques)

int TreeIdSolver_Vereshchagin::CartToJnt(const JntArray& q, const JntArray& q_dot, JntArray& q_dotdot, const Jacobian& alfa, const JntArray& beta, const Wrenches& f_ext, JntArray &torques)
{
    //Check sizes always
    if (q.rows() != nj || q_dot.rows() != nj || q_dotdot.rows() != nj || torques.rows() != nj)// || f_ext.size() != ns)
    {
        std::cout << "Error 1" << std::endl;
        return -1;
    }
    if (alfa.columns() != nc || beta.rows() != nc)
        //  if (alfas.size() != nc || betas.size() != nc) // number of constraints should also become a vector. because nc defines a number of columns in constraint jacobian matrix A(alfa)
    {
        std::cout << "Error 2" << std::endl;
        return -2;
    }




    //do an upward recursion for position and velocities
    this->initial_upwards_sweep(q, q_dot, q_dotdot, f_ext);
    //do an inward recursion for inertia, forces and constraints
    //    this->downwards_sweep(alfa, torques);
    //Solve for the constraint forces
    //    this->constraint_calculation(beta);
    //do an upward recursion to propagate the result
    //    this->final_upwards_sweep(q_dotdot, torques);
    return 0;
}

void TreeIdSolver_Vereshchagin::initial_upwards_sweep(const JntArray &q, const JntArray &qdot, const JntArray &qdotdot, const Wrenches& f_ext)
{
    F_total = Frame::Identity();
    Twist vj;
    Twist aj;
    unsigned int segmentNr = 0;
    //std::vector<segment_info> tempParentInfo = std::vector<segment_info > (ns + 1, segment_info(nc));
    //Does this rely on weak ordering assumption of the maps and does it really matter?

    /*
    //Breadth first search
    for (SegmentMap::const_iterator it = tree.getSegments().begin(); it != tree.getSegments().end(); it++)
    {

        for (unsigned int i = 0; i < it->second.children.size(); i++) //get children of the currentSegment
        {

            const Segment& childSegment = it->second.children[i]->second.segment; //segment id of the child i
            segment_info& childSegmentState = segmentResults[segmentNr + 1];

            childSegmentState.segmentName = childSegment.getName();
            std::cout << "Segment name in of current segment" << childSegmentState.segmentName << std::endl;
            //std::cout << "Child iteration number " << i << std::endl;
            childSegmentState.F = childSegment.pose(q(it->second.children[i]->second.q_nr));
            
            //   childSegmentState.F_base = currentFrame * childSegmentState.F;
            std::cout << "F link local:  " << std::endl << childSegmentState.F << std::endl;

            //The velocity due to the joint motion of the segment expressed in the segments reference frame (tip): alsno know as angular velocity
            vj = childSegmentState.F.M.Inverse(childSegment.twist(q(it->second.children[i]->second.q_nr), qdot(it->second.children[i]->second.q_nr))); //use feature of KDL tree joint numbering
            aj = childSegmentState.F.M.Inverse(childSegment.twist(q(it->second.children[i]->second.q_nr), qdot(it->second.children[i]->second.q_nr))); //when creating a tree through addsegment every iteration starting from
                                                                                                                                                       //root joint numbers will have the same value as the iteration index.
                                                                                                                                                       //Therefore, one can count that q_in, qdot_in, qdotdot_in array indeces will correspond to the right joints in tree.
            //The unit velocity due to the joint motion of the segment expressed in the segments reference frame (tip)
            childSegmentState.Z = childSegmentState.F.M.Inverse(childSegment.twist(q(it->second.children[i]->second.q_nr), 1.0));
            //Put Z in the joint root reference frame:
            childSegmentState.Z = childSegmentState.F * childSegmentState.Z;

            if (it == tree.getRootSegment()) // for children of the tree root
            {
                childSegmentState.v = vj;
                childSegmentState.A = childSegmentState.F.M.Inverse(acc_root);
                childSegmentState.F_base = childSegmentState.F;
//                std::cout << "I am a child of the root F_base  " << std::endl<< childSegmentState.F_base << std::endl;
//                std::cout << "I am a child of the root F_local " << std::endl<< childSegmentState.F << std::endl;
            }

            /*
              
               //The total velocity of the segment expressed in the the segments reference frame (tip)
               if (i != 0)
               {
                   s.v = s.F.Inverse(segmentResults[i].v) + vj; // Azamat: recursive velocity of each link in segment frame

                   //s.A=s.F.Inverse(results[i].A)+aj;
                   s.A = s.F.M.Inverse(segmentResults[i].A);
               }
           
               //c[i] = cj + v[i]xvj (remark: cj=0, since our S is not time dependent in local coordinates)
               //The velocity product acceleration
               //std::cout << i << " Initial upward" << s.v << std::endl;
               s.C = s.v*vj; //This is a cross product Azamat: cartesian space BIAS acceleration in local link coord.

               //s.A+=s.C;
               //std::cout<<"segment"<<i<<" acceleration: "<<F_total.M*s.A<<std::endl;

               //Put C in the joint root reference frame
               s.C = s.F * s.C; //+F_total.M.Inverse(acc_root));
               //The rigid body inertia of the segment, expressed in the segments reference frame (tip)
               s.H = segment.getInertia();

               //wrench of the rigid body bias forces and the external forces on the segment (in body coordinates, tip)
               //Azamat: external forces are taken into account through s.U. Check why s.U is given as below
               Wrench FextLocal = F_total.M.Inverse() * f_ext[i];
               s.U = s.v * (s.H * s.v) - FextLocal; //f_ext[i];
  
            childSegmentState.iteratedOn = 1;
            segmentNr++;
        }
    
    }
       * */

    //first get the serial chains and calculate for them and skip on the others which were iterated on
    for (unsigned int i = 0 ; i < chainInTree.size(); i++)
    {

        for  (unsigned int j = 0; j < chainInTree[i].getNrSegments(); j++)
        {
         //   chainInTree[i].getSegment(j).
        }


    }

    for(std::vector<segment_info>::const_iterator resultsIter = segmentResults.begin(); resultsIter != segmentResults.end(); resultsIter++)
        {
                std::cout << "I have been through this "<< resultsIter->segmentName <<" Pose "<<resultsIter->iteratedOn<< std::endl;
                
                //std::cout << "F-base "<< std::endl << resultsIter->F_base << std::endl;
        }



}






/*
void TreeIdSolver_Vereshchagin::initial_upwards_sweep(const JntArray &q, const JntArray &qdot, const JntArray &qdotdot, const Wrenches& f_ext)
{
    F_total = Frame::Identity();
    Twist vj;
    Twist aj;

    unsigned int jointNr = 0;
    unsigned int segmentNr = 0;


    for (unsigned int chainNr = 0; chainNr < chainInTree.size(); chainNr++)
    {
        unsigned int i = segmentNr;

        //std::cout << "Number of segments in current chain" << chainInTree[chainNr].getNrOfSegments() << std::endl;
        for (i; i < chainInTree[chainNr].getNrOfSegments(); i++)
        {
            const Segment& segment = chainInTree[chainNr].getSegment(i); //initialize a reference to a segment in a chain
            std::cout << "Segment name in current chain " << segment.getName() << std::endl;
            segment_info& s = segmentResults[i + 1]; //initialize a reference to segment_info reference to a element of segmentResults vector
            if (s.iteratedOn == 0)
            {
                s.segmentName = segment.getName(); // assign info on current segment in the chain to a segment_info
                std::cout << "Not iterated segment name in current chain " << s.segmentName << std::endl;
                //The pose between the joint root and the segment tip (tip expressed in joint root coordinates)
                s.F = segment.pose(q(jointNr)); // Azamat: X pose of each link in link coord system

                F_total = F_total * s.F; // Azamat: X pose of the each link in root coord system
                // std::cout << "F_total:  " << F_total << std::endl;

                s.F_base = F_total; // Azamat: X pose of the each link in root coord system for getter functions

                //The velocity due to the joint motion of the segment expressed in the segments reference frame (tip)
                vj = s.F.M.Inverse(segment.twist(q(jointNr), qdot(jointNr))); //Azamat: XDot of each link
                aj = s.F.M.Inverse(segment.twist(q(jointNr), qdotdot(jointNr))); //Azamat: XDotDot of each link

                //The unit velocity due to the joint motion of the segment expressed in the segments reference frame (tip)
                s.Z = s.F.M.Inverse(segment.twist(q(jointNr), 1.0));
                //Put Z in the joint root reference frame:
                s.Z = s.F * s.Z;
                //  std::cout<<"q: "<<q(j)<<std::endl;
                //The total velocity of the segment expressed in the the segments reference frame (tip)
                if (i != 0)
                {
                    s.v = s.F.Inverse(segmentResults[i].v) + vj; // Azamat: recursive velocity of each link in segment frame

                    //s.A=s.F.Inverse(results[i].A)+aj;
                    s.A = s.F.M.Inverse(segmentResults[i].A);
                }
                else
                {
                    s.v = vj;
                    s.A = s.F.M.Inverse(acc_root);
                }
                //c[i] = cj + v[i]xvj (remark: cj=0, since our S is not time dependent in local coordinates)
                //The velocity product acceleration
                //std::cout << i << " Initial upward" << s.v << std::endl;
                s.C = s.v*vj; //This is a cross product Azamat: cartesian space BIAS acceleration in local link coord.

                //s.A+=s.C;
                //std::cout<<"segment"<<i<<" acceleration: "<<F_total.M*s.A<<std::endl;

                //Put C in the joint root reference frame
                s.C = s.F * s.C; //+F_total.M.Inverse(acc_root));
                //The rigid body inertia of the segment, expressed in the segments reference frame (tip)
                s.H = segment.getInertia();

                //wrench of the rigid body bias forces and the external forces on the segment (in body coordinates, tip)
                //Azamat: external forces are taken into account through s.U. Check why s.U is given as below
                Wrench FextLocal = F_total.M.Inverse() * f_ext[i];
                s.U = s.v * (s.H * s.v) - FextLocal; //f_ext[i];
                s.iteratedOn = 1;
                segmentNr = tree.getNrOfSegments() - i;
                std::cout << segmentNr << std::endl;
                if (segment.getJoint().getType() != Joint::None)
                    jointNr++;
            }
        }
    }


}
 */
/*
void TreeIdSolver_Vereshchagin::downwards_sweep(const Jacobian& alfa, const JntArray & torques)
{
    unsigned int j = nj - 1;
    for (int i = ns; i >= 0; i--)
    {
        //Get a handle for the segment we are working on.
        segment_info& s = results[i];
        //For segment N,
        //tilde is in the segment refframe (tip, not joint root)
        //without tilde is at the joint root (the childs tip!!!)
        //P_tilde is the articulated body inertia
        //R_tilde is the sum of external and coriolis/centrifugal forces
        //M is the (unit) acceleration energy already generated at link i
        //G is the (unit) magnitude of the constraint forces at link i
        //E are the (unit) constraint forces due to the constraints
        if (i == ns)
        {
            s.P_tilde = s.H;
            s.R_tilde = s.U;
            s.M.setZero();
            s.G.setZero();
            //changeBase(alfa_N,F_total.M.Inverse(),alfa_N2);
            for (unsigned int r = 0; r < 3; r++)
                for (unsigned int c = 0; c < nc; c++)
                {
                    //copy alfa constrain force matrix in E~
                    s.E_tilde(r, c) = alfa(r + 3, c);
                    s.E_tilde(r + 3, c) = alfa(r, c);
                }
            //Change the reference frame of alfa to the segmentN tip frame
            //F_Total holds end effector frame, if done per segment bases then constraints could be extended to all segments
            Rotation base_to_end = F_total.M.Inverse();
            for (unsigned int c = 0; c < nc; c++)
            {
                Wrench col(Vector(s.E_tilde(3, c), s.E_tilde(4, c), s.E_tilde(5, c)),
                           Vector(s.E_tilde(0, c), s.E_tilde(1, c), s.E_tilde(2, c)));
                col = base_to_end*col;
                s.E_tilde.col(c) << Vector3d::Map(col.torque.data), Vector3d::Map(col.force.data);
            }
        }
        else
        {
            //For all others:
            //Everything should expressed in the body coordinates of segment i
            segment_info& child = results[i + 1];
            //Copy PZ into a vector so we can do matrix manipulations, put torques above forces
            Vector6d vPZ;
            vPZ << Vector3d::Map(child.PZ.torque.data), Vector3d::Map(child.PZ.force.data);
            Matrix6d PZDPZt = (vPZ * vPZ.transpose()).lazy();
            PZDPZt /= child.D;

            //equation a) (see Vereshchagin89) PZDPZt=[I,H;H',M]
            //Azamat:articulated body inertia as in Featherstone (7.19)
            s.P_tilde = s.H + child.P - ArticulatedBodyInertia(PZDPZt.corner < 3, 3 > (BottomRight), PZDPZt.corner < 3, 3 > (TopRight), PZDPZt.corner < 3, 3 > (TopLeft));
            //equation b) (see Vereshchagin89)
            //Azamat: bias force as in Featherstone (7.20)
            s.R_tilde = s.U + child.R + child.PC + (child.PZ / child.D) * child.u;
            //equation c) (see Vereshchagin89)
            s.E_tilde = child.E;

            //Azamat: equation (c) right side term
            s.E_tilde -= (vPZ * child.EZ.transpose()).lazy() / child.D;

            //equation d) (see Vereshchagin89)
            s.M = child.M;
            //Azamat: equation (d) right side term
            s.M -= (child.EZ * child.EZ.transpose()).lazy() / child.D;

            //equation e) (see Vereshchagin89)
            s.G = child.G;
            Twist CiZDu = child.C + (child.Z / child.D) * child.u;
            Vector6d vCiZDu;
            vCiZDu << Vector3d::Map(CiZDu.rot.data), Vector3d::Map(CiZDu.vel.data);
            s.G += (child.E.transpose() * vCiZDu).lazy();
            // std::cout<<"C: "<<child.D<<std::endl;
            // std::cout<<"u: "<<child.u<<std::endl;
        }
        if (i != 0)
        {
            //Transform all results to joint root coordinates of segment i (== body coordinates segment i-1)
            //equation a)
            s.P = s.F * s.P_tilde;
            //equation b)
            s.R = s.F * s.R_tilde;
            //equation c), in matrix: torques above forces, so switch and switch back
            for (unsigned int c = 0; c < nc; c++)
            {
                Wrench col(Vector(s.E_tilde(3, c), s.E_tilde(4, c), s.E_tilde(5, c)),
                           Vector(s.E_tilde(0, c), s.E_tilde(1, c), s.E_tilde(2, c)));
                col = s.F*col;
                s.E.col(c) << Vector3d::Map(col.torque.data), Vector3d::Map(col.force.data);
            }

            //needed for next recursion
            s.PZ = s.P * s.Z;
            s.D = dot(s.Z, s.PZ);
            s.PC = s.P * s.C;

            //u=(Q-Z(R+PC)=sum of external forces along the joint axes,
            //R are the forces comming from the children,
            //Q is taken zero (do we need to take the previous calculated torques?

            //Azamat: projection of coriolis and centrepital forces into joint subspace (0 0 Z)
            s.totalBias = -dot(s.Z, s.R + s.PC);
            s.u = torques(j) + s.totalBias;

#ifdef DEBUGGER
            printf("Downwar sweep joint, s.u,  torques(j) %d                  %f                    %f\n", j, s.u, torques(j));
            std::cout << "totalBias" << i << ": " << s.totalBias << std::endl;
            std::cout << "s.u" << i << ": " << s.u << std::endl;
            std::cout << "s.R = " << s.R << std::endl;
            std::cout << "s.PC = " << s.PC << std::endl;
            std::cout << "-dot(s.Z,s.R+s.PC)" << -dot(s.Z, s.R + s.PC) << std::endl;
            std::cout << "s.u = " << s.u << std::endl;
#endif DEBUGGER
            // Azamat: Total forces on the joint (comes from the joint torques (generelazed forces) and coriolis/centrepital)
            //printf("Joint#           generalized force          s.R            s.PC\n");
            //printf("%d                  %f                    %f              %f\n",j, s.u, dot(s.Z,s.R), dot(s.Z,s.PC));

            //s.u=torques(j)-dot(s.Z,s.R);

            //Matrix form of Z, put rotations above translations
            Vector6d vZ;
            vZ << Vector3d::Map(s.Z.rot.data), Vector3d::Map(s.Z.vel.data);
            s.EZ = (s.E.transpose() * vZ).lazy();

            if (tree.getSegment(i - 1).getJoint().getType() != Joint::None)
                j--;
        }

            
        std::cout<<"E~ "<<i<<": "<<s.E_tilde<<std::endl;
        std::cout<<"R~ "<<i<<": "<<s.R_tilde<<std::endl;
        std::cout<<"M  "<<i<<": "<<s.M<<std::endl;
        std::cout<<"G  "<<i<<": "<<s.G<<std::endl;
 *
        std::cout<<"E"<<i<<": "<<s.E<<std::endl;
        std::cout<<"Z: "<<s.Z<<std::endl;
        std::cout<<"D: "<<s.D<<std::endl;
        std::cout<<"PZ: "<<s.PZ<<std::endl;
        std::cout<<"E'Z: "<<s.EZ<<std::endl;
        std::cout<<"G: "<<s.G<<std::endl;
        std::cout<<"M: "<<s.M<<std::endl;
             
            
        std::cout<<"For segment "<<i<<std::endl;
        std::cout<<"D: "<<s.D<<std::endl;
        std::cout<<"E~: "<<s.E_tilde<<std::endl;
        std::cout<<"E: "<<s.E<<std::endl;

        std::cout<<"E: "<<s.E<<std::endl;
        std::cout<<"Z: "<<s.Z.rot<<s.Z.vel<<std::endl;
        Matrix6d tmp;
        tmp<<s.P_tilde.I,s.P_tilde.H,s.P_tilde.H.transpose(),s.P_tilde.M;
        std::cout<<"P~: \n"<<tmp<<std::endl;
        tmp<<s.P.I,s.P.H,s.P.H.transpose(),s.P.M;
        std::cout<<"P: \n"<<tmp<<std::endl;
             
    }
}

void TreeIdSolver_Vereshchagin::constraint_calculation(const JntArray & beta)
{
    //equation f) nu = M_0_inverse*(beta_N - E0_tilde`*acc0 - G0)
    //M_0_inverse, always nc*nc symmetric matrix
    //std::cout<<"M0: "<<results[0].M<<std::endl;
    //results[0].M-=MatrixXd::Identity(nc,nc);
    //std::cout<<"augmented M0: "<<results[0].M<<std::endl;


    //Azamat: Need to check ill conditions

    //M_0_inverse=results[0].M.inverse();
    svd_eigen_HH(results[0].M, Um, Sm, Vm, tmpm);
    //truncated svd, what would sdls, dls physically mean?
    for (unsigned int i = 0; i < nc; i++)
        if (Sm(i) < 1e-14)
            Sm(i) = 0.0;
        else
            Sm(i) = 1 / Sm(i);

    results[0].M = (Vm * Sm.asDiagonal()).lazy();
    M_0_inverse = (results[0].M * Um.transpose()).lazy();
    //results[0].M.ldlt().solve(MatrixXd::Identity(nc,nc),&M_0_inverse);
    //results[0].M.computeInverse(&M_0_inverse);
    //std::cout<<"inv(M0): "<<M_0_inverse<<std::endl;

    Vector6d acc;
    acc << Vector3d::Map(acc_root.rot.data), Vector3d::Map(acc_root.vel.data);
    nu_sum = -(results[0].E_tilde.transpose() * acc).lazy();

    //std::cout<<"E~0: "<<results[0].E_tilde.transpose()<<std::endl;
    // std::cout<<"E~0*acc0: "<<acc<<std::endl;
    //std::cout<<"E~0*acc0: "<<nu_sum<<std::endl;

    //nu_sum.setZero();
    nu_sum += beta.data;
    nu_sum -= results[0].G;
    //std::cout<<"G0: "<<results[0].G<<std::endl;

    //equation f) nu = M_0_inverse*(beta_N - E0_tilde`*acc0 - G0)
    nu = (M_0_inverse * nu_sum).lazy();

    //      std::cout<<"nu: "<<nu<<std::endl;
   
}


void TreeIdSolver_Vereshchagin::final_upwards_sweep(JntArray &q_dotdot, JntArray & torques)
{
    unsigned int j = 0;

    for (unsigned int i = 1; i <= ns; i++)
    {
        segment_info& s = results[i];
        //Calculation of joint and segment accelerations
        //equation g) qdotdot[i] = D^-1*(Q - Z'(R + P(C + acc[i-1]) + E*nu))
        // = D^-1(u - Z'(P*acc[i-1] + E*nu)
        Twist a_g;
        Twist a_p;
        if (i == 1)
        {
            a_p = acc_root;
        }
        else
        {
            a_p = results[i - 1].acc;
        }

        //The contribution of the constraint forces at segment i
        Vector6d tmp = s.E*nu;
        //std::cout<<"E"<<i<<": "<<s.E<<std::endl;
        Wrench constraint_force = Wrench(Vector(tmp(3), tmp(4), tmp(5)),
                                         Vector(tmp(0), tmp(1), tmp(2)));
       

        //Azamat: acceleration components are also computed
        //Contribution of the acceleration of the parent (i-1)
        Wrench parent_force = s.P*a_p;
        double parent_forceProjection = -dot(s.Z, parent_force);
        double parentAccComp = parent_forceProjection / s.D;

        //The constraint force and acceleration force projected on the joint axes -> axis torque/force
        double constraint_torque = -dot(s.Z, constraint_force);
        //The result should be the torque at this joint

        torques(j) = constraint_torque;
        //s.constAccComp = torques(j) / s.D;
        s.constAccComp = constraint_torque / s.D;
        s.nullspaceAccComp = s.u / s.D;

        q_dotdot(j) = (s.nullspaceAccComp + parentAccComp + s.constAccComp);

#ifdef DEBUGGER
        // printf("Torque contributions U - %f,  biasForces - %f, constraintTorque - %f,  parentForces - %f\n", s.u,  s.totalBias, constraint_torque,  parent_forceProjection);
        // printf("Articulated body inertia D - %f\n", s.D);
#endif DEBUGGER



        //Azamat: The projection on Z should be of sum of constraint forces and parent forces
        //q_dotdot(j) = (torques(j))/s.D;
        //Azamat: s.u is generalized forces in the joint coord. without constraints.
        //Azamat: s.u=torques(j)-dot(s.Z,s.R+s.PC);
        //printf("Total forces in joint space %f\n", (s.u-dot(s.Z,parent_force+torques(j)))); // Azamat: Have to put constraint forces in Wrench form and then add,
        //equation h) acc[i] = Fi*(acc[i-1] + Z[i]*qdotdot[i] + c[i]                        //Azamat: here torques variable represents constraint forces, check lines above
        //std::cout<<"Z"<<i<<": "<<s.Z<<std::endl;
        //std::cout<<"C"<<i<<": "<<s.C<<std::endl;

        s.acc = s.F.Inverse(a_p + s.Z * q_dotdot(j) + s.C); //Azamat: returns acceleration in link distal tip coordinates. For use needs to be transformed

#ifdef DEBUGGER
        std::cout << "a_p = " << a_p << std::endl;
        std::cout << "s_C = " << s.C << std::endl;
        std::cout << "s.Z*q_dotdot(j) =" << s.Z * q_dotdot(j) << std::endl;
        std::cout << "acceleration_local: " << s.acc << std::endl;
#endif DEBUGGER

        //q_dotdot(j) = (s.u+constraint_torque)/s.D;


        if (tree_in.getSegment(i - 1).getJoint().getType() != Joint::None)
            j++;
    }
    //std::cout<<"endpoint acceleration: "<<F_total.M*results[ns].acc<<std::endl;
    //std::cout<<"endpoint position: "<<F_total<<std::endl;
}

 */

/*
void TreeIdSolver_Vereshchagin::getLinkCartesianPose(Frames& x_base)
{
    for (int i = 0; i < ns; i++)
    {
        x_base[i] = results[i + 1].F_base;
    }
    return;
}

void TreeIdSolver_Vereshchagin::getLinkCartesianVelocity(Twists& xDot_base)
{

    for (int i = 0; i < ns; i++)
    {
        xDot_base[i] = results[i + 1].F_base.M * results[i + 1].v;
    }
    return;
}

void TreeIdSolver_Vereshchagin::getLinkCartesianAcceleration(Twists& xDotDot_base)
{

    for (int i = 0; i < ns; i++)
    {
        xDotDot_base[i] = results[i + 1].F_base.M * results[i + 1].acc;
        //std::cout << "XDotDot_base[i] " << xDotDot_base[i] << std::endl;
    }
    return;
}

void TreeIdSolver_Vereshchagin::getLinkPose(Frames& x_local)
{
    for (int i = 0; i < ns; i++)
    {
        x_local[i] = results[i + 1].F;
    }
    return;
}

void TreeIdSolver_Vereshchagin::getLinkVelocity(Twists& xDot_local)
{
    for (int i = 0; i < ns; i++)
    {
        xDot_local[i] = results[i + 1].v;
    }
    return;

}

void TreeIdSolver_Vereshchagin::getLinkAcceleration(Twists& xDotdot_local)
{
     for (int i = 0; i < ns; i++)
    {
        xDotdot_local[i] = results[i + 1].acc;
    }
    return;

}

void TreeIdSolver_Vereshchagin::getJointBiasAcceleration(JntArray& bias_q_dotdot)
{
    for (int i = 0; i < ns; i++)
    {
        //this is only force
        double tmp = results[i + 1].totalBias;
        //this is accelleration
        bias_q_dotdot(i) = tmp / results[i + 1].D;

        //s.totalBias = - dot(s.Z, s.R + s.PC);
        //std::cout << "totalBiasAccComponent" << i << ": " << bias_q_dotdot(i) << std::endl;
        //bias_q_dotdot(i) = s.totalBias/s.D

    }
    return;

}

void TreeIdSolver_Vereshchagin::getJointConstraintAcceleration(JntArray& constraint_q_dotdot)
{
    for (int i = 0; i < ns; i++)
    {
        constraint_q_dotdot(i) = results[i + 1].constAccComp;
        //double tmp = results[i + 1].u;
        //s.u = torques(j) + s.totalBias;
        // std::cout << "s.constraintAccComp" << i << ": " << results[i+1].constAccComp << std::endl;
        //nullspace_q_dotdot(i) = s.u/s.D

    }
    return;


}

//Check the name it does not seem to be appropriate

void TreeIdSolver_Vereshchagin::getJointNullSpaceAcceleration(JntArray& nullspace_q_dotdot)
{
    for (int i = 0; i < ns; i++)
    {
        nullspace_q_dotdot(i) = results[i + 1].nullspaceAccComp;
        //double tmp = results[i + 1].u;
        //s.u = torques(j) + s.totalBias;
        //std::cout << "s.nullSpaceAccComp" << i << ": " << results[i+1].nullspaceAccComp << std::endl;
        //nullspace_q_dotdot(i) = s.u/s.D

    }
    return;


}

//This is not only a bias force energy but also includes generalized forces
//change type of parameter G
//this method should retur array of G's

void TreeIdSolver_Vereshchagin::getLinkBiasForceAcceleratoinEnergy(Eigen::VectorXd& G)
{
    for (int i = 0; i < ns; i++)
    {
        G = results[i + 1].G;
        //double tmp = results[i + 1].u;
        //s.u = torques(j) + s.totalBias;
        //std::cout << "s.G " << i << ":  " << results[i+1].G << std::endl;
        //nullspace_q_dotdot(i) = s.u/s.D

    }
    return;

}

//this method should retur array of R's

void TreeIdSolver_Vereshchagin::getLinkBiasForceMatrix(Wrenches& R_tilde)
{
    for (int i = 0; i < ns; i++)
    {
        R_tilde[i] = results[i + 1].R_tilde;
        //Azamat: bias force as in Featherstone (7.20)
        //s.R_tilde = s.U + child.R + child.PC + child.PZ / child.D * child.u;
        std::cout << "s.R_tilde " << i << ":  " << results[i + 1].R_tilde << std::endl;
    }
    return;
}

//this method should retur array of M's

void TreeIdSolver_Vereshchagin::getLinkUnitForceAccelerationEnergy(Eigen::MatrixXd& M)
{


}

//this method should retur array of E_tilde's

void TreeIdSolver_Vereshchagin::getLinkUnitForceMatrix(Matrix6Xd& E_tilde)
{


}

 */

}//namespace

