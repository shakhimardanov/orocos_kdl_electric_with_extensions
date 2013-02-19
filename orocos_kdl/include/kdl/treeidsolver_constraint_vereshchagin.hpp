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

#ifndef KDL_TREE_IDSOLVER_VERESHCHAGIN_HPP
#define KDL_TREE_IDSOLVER_VERESHCHAGIN_HPP

#include "treeidsolver.hpp"
#include "frames.hpp"
#include "articulatedbodyinertia.hpp"

namespace KDL{
    /**
     * \brief Dynamics calculations by constraints based on Vereshchagin 1989.
     * 
     * For a tree
     */
    struct TreeConstraint{
        std segment_name;
        Vector<6,1,double> Ai;
    }

    class TreeIdSolver_Constraint_Vereshchagin{
    public:
        /**
         * Constructor for the solver, it will allocate all the necessary memory
         * \param tree The kinematic tree to calculate the inverse dynamics for, an internal copy will be made.
         * \param root_acc The acceleration vector of the root to use during the calculation.(most likely contains gravity)
         *
         */
        TreeIdSolver_Constraint_Vereshchagin(const Tree& tree,Twist root_acc,unsigned int nc);
        ~TreeIdSolver_Constraint_Vereshchagin(){};
        
        /**
         * Function to calculate from Cartesian forces to joint torques.
         * Input parameters;
         * \param q The current joint positions
         * \param q_dot The current joint velocities
         * \param f_ext The external forces (no gravity) on the segments
         * Output parameters:
         * \param q_dotdot The joint accelerations
         * \param torques the resulting torques for the joints
         */
        int CartToJnt(const JntArray &q, const JntArray &q_dot, JntArray &q_dotdot, const Jacobian& alfa, const JntArray& beta, const Wrenches& f_ext,JntArray &torques);
        
    private:
        //Functions to calculate velocity, propagated inertia, propagated bias forces, constraint forces and accelerations
        void initial_upwards_sweep(const JntArray &q, const JntArray &q_dot,const JntArray &q_dotdot, const Wrenches& f_ext);
        void downwards_sweep(const Jacobian& alfa,const JntArray& torques);
        void constraint_calculation(const JntArray& beta);
        void final_upwards_sweep(JntArray &q_dotdot, JntArray &torques);

        Tree tree;
        unsigned int nj;
        unsigned int ns;
        unsigned int nc;
        Twist acc_root;

        typedef Eigen::Matrix<double,6,1> Vector6d;
        typedef Eigen::Matrix<double,6,6> Matrix6d;
        typedef Eigen::Matrix<double,6,Eigen::Dynamic> Matrix6Xd;
        
        struct segment_info{
            Frame F;//Pose
            Twist Z;//Unit twist
            Twist v;//twist
            Twist acc;//acceleration twist
            Wrench U;//wrench p of the bias forces 
            Wrench R;//wrench p of the bias forces 
            Wrench R_tilde;//vector of wrench p of the bias forces (new) in matrix form
            Twist C;//constraint
            Twist A;//constraint
            ArticulatedBodyInertia H;//I (expressed in 6*6 matrix) 
            ArticulatedBodyInertia P;//I (expressed in 6*6 matrix) 
            ArticulatedBodyInertia P_tilde;//I (expressed in 6*6 matrix) 
            Wrench PZ;//vector U[i] = I_A[i]*S[i]
            Wrench PC;//vector E[i] = I_A[i]*c[i]
            double D;//vector D[i] = S[i]^T*U[i]
            Matrix6Xd E;//matrix with virtual unit constraint force due to acceleration constraints
            Matrix6Xd E_tilde;
            Eigen::MatrixXd M;//acceleration energy already generated at link i
            Eigen::VectorXd G;//magnitude of the constraint forces already generated at link i
            Eigen::VectorXd EZ;//K[i] = Etiltde'*Z
            double u;//vector u[i] = torques(i) - S[i]^T*(p_A[i] + I_A[i]*C[i])
            segment_info(unsigned int nc){
                E.resize(6,nc);
                E_tilde.resize(6,nc);
                G.resize(nc);
                M.resize(nc,nc);
                EZ.resize(nc);
                E.setZero();
                E_tilde.setZero();
                M.setZero();
                G.setZero();
                EZ.setZero();
            };
        };

        std::vector<segment_info> results;
        
        Jacobian alfa_N,alfa_N2;
        Eigen::MatrixXd M_0_inverse,Um,Vm;
        JntArray beta_N;
        Eigen::VectorXd nu,nu_sum,Sm,tmpm;
        Wrench qdotdot_sum;
        
        Frame F_total;
    };
}

#endif
