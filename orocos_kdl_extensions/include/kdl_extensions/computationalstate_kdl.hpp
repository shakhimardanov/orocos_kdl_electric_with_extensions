/****************************************************************************** 
*           This file is part of the Geometric harmonization project          *
*                                                                             *
*                            (C) 2014 Azamat Shakhimardanov                   *
*                               Herman Bruyninckx                             *
*                        azamat.shakhimardanov@mech.kuleuven.be               *                              
*                    Department of Mechanical Engineering,                    *
*                   Katholieke Universiteit Leuven, Belgium.                  *
*                                                                             *
*       You may redistribute this software and/or modify it under either the  *
*       terms of the GNU Lesser General Public License version 2.1 (LGPLv2.1  *
*       <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>) or (at your *
*       discretion) of the Modified BSD License:                              *
*       Redistribution and use in source and binary forms, with or without    *
*       modification, are permitted provided that the following conditions    *
*       are met:                                                              *
*       1. Redistributions of source code must retain the above copyright     *
*       notice, this list of conditions and the following disclaimer.         *
*       2. Redistributions in binary form must reproduce the above copyright  *
*       notice, this list of conditions and the following disclaimer in the   *
*       documentation and/or other materials provided with the distribution.  *
*       3. The name of the author may not be used to endorse or promote       *
*       products derived from this software without specific prior written    *
*       permission.                                                           *
*       THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR  *
*       IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED        *
*       WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE    *
*       ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,*
*       INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES    *
*       (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS       *
*       OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) *
*       HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,   *
*       STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING *
*       IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE    *
*       POSSIBILITY OF SUCH DAMAGE.                                           *
*                                                                             *
*******************************************************************************/

#ifndef COMPUTATIONALSTATE_KDLTYPES_HPP
#define	COMPUTATIONALSTATE_KDLTYPES_HPP

#include <kdl_extensions/geometric_semantics_kdl.hpp>
#include <kdl/frames_io.hpp>


// TODO: maybe we should introduce domain independent
// representation of a computational state and instantiate for the specific domain
// The specific case could looks as below.

namespace kdle
{

    //TODO: consider whether link local and global computational states
    // should be represented in the single data type, as Herman suggested

    //TODO: These has to be deprecated in the favor of Global and Local?
    class SegmentState
    {
    public:
        SegmentState();
        SegmentState(const SegmentState& copy);
        SegmentState & operator=(const SegmentState& copy);
        bool operator==(const SegmentState& instance);
        bool operator!=(const SegmentState& instance);

        KDL::Frame X;
        KDL::Frame Xtotal; //should this be in here or be computed in "accumulate" functor
        KDL::Twist Xdot;
        KDL::Twist Xdotdot;
    //    KDL::Wrench Fext; // Fext is independent of any other segment's Fext
        KDL::Wrench F;
        KDL::Twist Z; //supporting/driving joint unit twist/projection/Dof
        KDL::Twist Vj;
        unsigned int jointIndex; // supporting/driving joint name/index
        std::string jointName;
        std::string segmentName;
        virtual ~SegmentState();
    };

    //immutable state

    class JointState
    {
    public:
        JointState();
        JointState(const JointState& copy);
        JointState & operator=(const JointState& copy);
        double q;
        double qdot;
        double qdotdot;
        double torque;
        KDL::Wrench Fext; // this is a hack and should be put somewhere else
        unsigned int jointIndex; //joint name/index
        std::string jointName;
        virtual ~JointState();
    };

    enum class StateSpaceType :char
    {
        JointSpace = 'J',
        CartesianSpace = 'C',
        SensorSpace = 'S',
    };

    template <typename PoseT, typename TwistT, StateSpaceType spaceT>
    class ComputationalState;

    template <>
    class ComputationalState<grs::Pose<KDL::Vector, KDL::Rotation>, grs::Twist<KDL::Vector, KDL::Vector>, StateSpaceType::CartesianSpace>
    {
     public:
        ComputationalState()=default;
//        ComputationalState(const ComputationalState& copy);
//        ComputationalState & operator=(const ComputationalState& copy);
//        bool operator==(const ComputationalState& instance);
//        bool operator!=(const ComputationalState& instance);

        grs::Pose<KDL::Vector, KDL::Rotation> X;
        grs::Twist<KDL::Vector, KDL::Vector> Xdot;
        grs::Twist<KDL::Vector, KDL::Vector> Xdotdot;
        grs::Twist<KDL::Vector, KDL::Vector> UnitXdot;
        std::string segmentName;
        ~ComputationalState()=default;   
    };

    template <>
    class ComputationalState<grs::Pose<KDL::Vector, KDL::Rotation>, grs::Twist<KDL::Vector, KDL::Vector>, StateSpaceType::JointSpace >
    {
     public:
        ComputationalState()=default;
//        ComputationalState(const ComputationalState& copy);
//        ComputationalState & operator=(const ComputationalState& copy);
//        bool operator==(const ComputationalState& instance);
//        bool operator!=(const ComputationalState& instance);

        std::vector<double> q;
        double qdot;
        double qdotdot;
        double torque;
        std::string jointName;
        ~ComputationalState()=default;   
    };

};


#endif	/* COMPUTATIONALSTATE_KDLTYPES_HPP */

