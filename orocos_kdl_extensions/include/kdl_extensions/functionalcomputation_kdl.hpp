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

#ifndef FUNCTIONALCOMPUTATION_KDLTYPES_HPP
#define	FUNCTIONALCOMPUTATION_KDLTYPES_HPP


#include <kdl/tree.hpp>
#include <kdl_extensions/computationalstate_kdl.hpp>
#include <kdl_extensions/functionalcomputation.hpp>
#include <kdl_extensions/chain_geometric_primitives.hpp>


namespace kdle
{
    //operation tags

    class poseOperationTag
    {
    };

    class twistOperationTag : public poseOperationTag
    {
    };

    class accelerationTwistOperationTag : public twistOperationTag
    {
    };

    class spatialForceOperationTag : public accelerationTwistOperationTag
    {
    };

    class wrenchOperationTag : public spatialForceOperationTag
    {
    };

    class inertiaOperationTag
    {
    };

    typedef poseOperationTag pose;
    typedef twistOperationTag twist;
    typedef accelerationTwistOperationTag accTwist;
    typedef spatialForceOperationTag force;
    typedef wrenchOperationTag wrench;
    typedef inertiaOperationTag inertia;

    typedef std::map<std::string, KDL::TreeElement >::const_iterator kdl_tree_iterator;

    template <typename Iterator>//, typename Operation> //accumulation operation (* or +)
    class accumulate
    {
        public:

            enum
            {
                NumberOfParams = 3
            };
            typedef SegmentState ReturnType;
            typedef Iterator Param1T;
            typedef JointState Param2T;
            typedef SegmentState Param3T;

            accumulate()
            {
            };
            //this needs to be changes, currently this is just overwritten

            accumulate(ParameterTypeQualifier<Param3T>::RefToConstT initialValue)
            {
                a_segmentState = initialValue;
            };

            inline ReturnType const& operator()(typename ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
                                                ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
                                                ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState1)
            {
                a_segmentState = p_segmentState1;
                a_segmentState.Xtotal = a_segmentState.Xtotal * p_segmentState1.X;
                return a_segmentState;
            };
        private:
            SegmentState a_segmentState;
    };

    template<typename Iterator, typename OperationTagT>
    class transform;

    template<>
    class transform<kdl_tree_iterator, pose>
    {
        public:

            enum
            {
                NumberOfParams = 5
            };
            typedef SegmentState ReturnType;
            typedef kdl_tree_iterator Param1T;
            typedef JointState Param2T;
            typedef SegmentState Param3T;
            typedef Param2T Param4T;
            typedef Param3T Param5T;

            inline ReturnType const& operator()(ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
                                                ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
                                                ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState)
            {
                //check for joint type None should be tree serialization function.
                a_segmentState.Xdot = p_segmentState.Xdot;
                a_segmentState.Xdotdot = p_segmentState.Xdotdot;
                a_segmentState.F = p_segmentState.F;

                a_segmentState.X = segmentId->second.segment.pose(p_jointState.q);
                a_segmentState.Xtotal = p_segmentState.Xtotal;
                a_segmentState.jointIndex = p_jointState.jointIndex;
                a_segmentState.jointName = p_jointState.jointName;
                a_segmentState.segmentName = segmentId->first;

                #ifdef VERBOSE_CHECK_KDLE
                    std::cout << "Inside pose operation 3 argument function call" << std::endl;
                    std::cout << "Inside pose operation Transform value" << std::endl << a_segmentState.X << std::endl;
                    std::cout << "Inside pose operation Twist value" << a_segmentState.Xtotal << std::endl;
                    std::cout << "Inside pose operation AccTwist value" << a_segmentState.Xdotdot << std::endl;
                    std::cout << "Inside pose operation Wrench value" << a_segmentState.F << std::endl;
                #endif
                return a_segmentState;

            };

            //returns current's updated state

            inline ReturnType const& operator()(ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
                                                ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
                                                ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState, //current's (initial) state
                                                ParameterTypeQualifier<Param4T>::RefToArgT p_jointState2,
                                                ParameterTypeQualifier<Param5T>::RefToConstT p_segmentState2) //current's child's  or parent's state
            {
                //check for joint type None should be tree serialization function.
                a_segmentState.Xdot = p_segmentState.Xdot;
                a_segmentState.Xdotdot = p_segmentState.Xdotdot;
                a_segmentState.F = p_segmentState.F;
                //        a_segmentState.Fext = p_segmentState.Fext;
                a_segmentState.X = segmentId->second.segment.pose(p_jointState.q);
                a_segmentState.Xtotal = p_segmentState.Xtotal;
                a_segmentState.jointIndex = p_jointState.jointIndex;
                a_segmentState.jointName = p_jointState.jointName;
                a_segmentState.segmentName = segmentId->first;

                #ifdef VERBOSE_CHECK_KDLE
                    std::cout << "Inside pose operation 5 argument function call" << std::endl;
                    std::cout << "Inside pose operation Transform value" << std::endl << a_segmentState.X << std::endl;
                    std::cout << "Inside pose operation Twist value" << a_segmentState.Xtotal << std::endl;
                    std::cout << "Inside pose operation AccTwist value" << a_segmentState.Xdotdot << std::endl;
                    std::cout << "Inside pose operation Wrench value" << a_segmentState.F << std::endl;
                #endif
                return a_segmentState;
            };


        private:
            ReturnType a_segmentState;

    };

    template<>
    class transform<kdl_tree_iterator, twist>
    {
        public:

            enum
            {
                NumberOfParams = 5
            };
            typedef SegmentState ReturnType;
            typedef kdl_tree_iterator Param1T;
            typedef JointState Param2T;
            typedef SegmentState Param3T;
            typedef Param2T Param4T;
            typedef Param3T Param5T;

            inline ReturnType const& operator()(ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
                                                ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
                                                ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState)
            {
                a_segmentState.X = p_segmentState.X;
                a_segmentState.Xdotdot = p_segmentState.Xdotdot;
                a_segmentState.F = p_segmentState.F;
                a_segmentState.Z = a_segmentState.X.M.Inverse(segmentId->second.segment.twist(p_jointState.q, 1.0));
                a_segmentState.Vj = a_segmentState.X.M.Inverse(segmentId->second.segment.twist(p_jointState.q, p_jointState.qdot));

                //do we check here for the index of a joint (whether the joint is first/root in the chain)
                //if so, somehow an information about whether a segment is root or not should be sent here
                a_segmentState.Xdot = a_segmentState.X.Inverse(p_segmentState.Xdot) + a_segmentState.Vj;
                #ifdef VERBOSE_CHECK_KDLE
                    std::cout << "Inside twist operation 3 argument function call" << std::endl;
                    std::cout << "Inside twist operation Transform value" << a_segmentState.X << std::endl;
                    std::cout << "Inside twist operation Twist value" << std::endl << a_segmentState.Xdot << std::endl;
                    std::cout << "Inside twist operation AccTwist value" << a_segmentState.Xdotdot << std::endl;
                    std::cout << "Inside twist operation Wrench value" << a_segmentState.F << std::endl;
                #endif
                return a_segmentState;
            };

            inline ReturnType const& operator()(ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
                                                ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
                                                ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState, //current's state (current's body force)
                                                ParameterTypeQualifier<Param4T>::RefToArgT p_jointState2,
                                                ParameterTypeQualifier<Param5T>::RefToConstT p_segmentState2) //current's child's state (child's total force)
            {
                a_segmentState.X = p_segmentState.X;
                a_segmentState.Xdotdot = p_segmentState.Xdotdot;
                a_segmentState.F = p_segmentState.F;
                a_segmentState.Z = a_segmentState.X.M.Inverse(segmentId->second.segment.twist(p_jointState.q, 1.0));
                a_segmentState.Vj = a_segmentState.X.M.Inverse(segmentId->second.segment.twist(p_jointState.q, p_jointState.qdot));

                //do we check here for the index of a joint (whether the joint is first/root in the chain)
                //if so, somehow an information about whether a segment is root or not should be sent here
                a_segmentState.Xdot = a_segmentState.X.Inverse(p_segmentState.Xdot) + a_segmentState.Vj;

                #ifdef VERBOSE_CHECK_KDLE
                    std::cout << "Inside twist operation 5 argument function call" << std::endl;
                    std::cout << "Inside twist operation Transform value" << a_segmentState.X << std::endl;
                    std::cout << "Inside twist operation Twist value" << std::endl << a_segmentState.Xdot << std::endl;
                    std::cout << "Inside twist operation AccTwist value" << a_segmentState.Xdotdot << std::endl;
                    std::cout << "Inside twist operation Wrench value" << a_segmentState.F << std::endl;
                #endif
                return a_segmentState;
            };

        private:
            ReturnType a_segmentState;
    };

    template<>
    class transform<kdl_tree_iterator, accTwist>
    {
        public:

            enum
            {
                NumberOfParams = 5
            };

            typedef SegmentState ReturnType;
            typedef kdl_tree_iterator Param1T;
            typedef JointState Param2T;
            typedef SegmentState Param3T;
            typedef Param2T Param4T;
            typedef Param3T Param5T;

            inline ReturnType const& operator()(ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
                                                ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
                                                ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState)
            {
                a_segmentState = p_segmentState;
                a_segmentState.Xdotdot = p_segmentState.X.Inverse(p_segmentState.Xdotdot) + p_segmentState.Z * p_jointState.qdotdot + p_segmentState.Xdot * p_segmentState.Vj;
                #ifdef VERBOSE_CHECK_KDLE
                    std::cout << "Inside accTwist operation 3 argument function call" << std::endl;
                    std::cout << "Inside acctwist operation Transform value" << a_segmentState.X << std::endl;
                    std::cout << "Inside acctwist operation Twist value" << a_segmentState.Xdot << std::endl;
                    std::cout << "Inside acctwist operation AccTwist value" << std::endl << a_segmentState.Xdotdot << std::endl;
//                    std::cout << "Inside acctwist operation Wrench value" << a_segmentState.F << std::endl;
                #endif
                return a_segmentState;
            };

            inline ReturnType const& operator()(ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
                                                ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
                                                ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState, //current's state (current's body force)
                                                ParameterTypeQualifier<Param4T>::RefToArgT p_jointState2,
                                                ParameterTypeQualifier<Param5T>::RefToConstT p_segmentState2) //current's child's state (child's total force)
            {
                a_segmentState = p_segmentState;
                a_segmentState.Xdotdot = p_segmentState.X.Inverse(p_segmentState.Xdotdot) + p_segmentState.Z * p_jointState.qdotdot + p_segmentState.Xdot * p_segmentState.Vj;

                #ifdef VERBOSE_CHECK_KDLE
                    std::cout << "Inside accTwist operation 5 argument function call" << std::endl;
                    std::cout << "Inside acctwist operation Transform value" << a_segmentState.X << std::endl;
                    std::cout << "Inside acctwist operation Twist value" << a_segmentState.Xdot << std::endl;
                    std::cout << "Inside acctwist operation AccTwist value" << std::endl << a_segmentState.Xdotdot << std::endl;
                    std::cout << "Inside acctwist operation Wrench value" << a_segmentState.F << std::endl;
//                    std::cout << "Inside acctwist operation Ext Wrench value" << a_segmentState.Fext << std::endl<< std::endl;
                #endif
                return a_segmentState;
            };

        private:
            ReturnType a_segmentState;

    };


    template<typename Iterator, typename OperationTagT>
    class balance;

    template<>
    class balance<kdl_tree_iterator, force>
    {
        public:

            enum
            {
                NumberOfParams = 5
            };

            typedef SegmentState ReturnType;
            typedef kdl_tree_iterator Param1T;
            typedef JointState Param2T;
            typedef SegmentState Param3T;
            typedef Param2T Param4T;
            typedef Param3T Param5T;

            inline ReturnType const& operator()(ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
                                                ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
                                                ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState)
            {
                a_segmentState = p_segmentState;
                a_segmentState.F = segmentId->second.segment.getInertia() * a_segmentState.Xdotdot + a_segmentState.Xdot * (segmentId->second.segment.getInertia() * a_segmentState.Xdot) - p_jointState.Fext;

                #ifdef VERBOSE_CHECK_KDLE
                    std::cout << "Inside wrench operation 3 argument function call " << std::endl;
                    std::cout << "Inside wrench operation Transform value " << a_segmentState.X << std::endl;
                    std::cout << "Inside wrench operation Twist value " << a_segmentState.Xdot << std::endl;
                    std::cout << "Inside wrench operation AccTwist value " << a_segmentState.Xdotdot << std::endl;
                    std::cout << "Inside wrench operation Wrench value " << std::endl << a_segmentState.F << std::endl << std::endl;
                    std::cout << "Inside wrench operation External wrench value " << std::endl << p_jointState.Fext << std::endl << std::endl;
                #endif
                return a_segmentState;
            };

            inline ReturnType const& operator()(ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
                                                ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
                                                ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState, //current's state (current's body force)
                                                ParameterTypeQualifier<Param4T>::RefToArgT p_jointState2,
                                                ParameterTypeQualifier<Param5T>::RefToConstT p_segmentState2) //current's child's state (child's total force)
            {
                a_segmentState = p_segmentState;
                a_segmentState.F = segmentId->second.segment.getInertia() * a_segmentState.Xdotdot + a_segmentState.Xdot * (segmentId->second.segment.getInertia() * a_segmentState.Xdot) - p_jointState.Fext;

                #ifdef VERBOSE_CHECK_KDLE
                    std::cout << "Inside wrench operation 5 argument fucntion call " << std::endl;
                    std::cout << "Inside wrench operation Transform value " << a_segmentState.X << std::endl;
                    std::cout << "Inside wrench operation Twist value " << a_segmentState.Xdot << std::endl;
                    std::cout << "Inside wrench operation AccTwist value " << a_segmentState.Xdotdot << std::endl;
                    std::cout << "Inside wrench operation Wrench value " << std::endl << a_segmentState.F << std::endl << std::endl;
                    std::cout << "Inside wrench operation External wrench value " << std::endl << p_jointState.Fext << std::endl << std::endl;
                #endif
                return a_segmentState;
            };

        private:
            ReturnType a_segmentState;

    };

    template<typename Iterator, typename OperationTagT>
    class project;

    template<>
    class project<kdl_tree_iterator, inertia>
    {
        public:

            enum
            {
                NumberOfParams = 5
            };

            typedef KDL::RigidBodyInertia ReturnType;
            typedef kdl_tree_iterator Param1T;
            typedef JointState Param2T;
            typedef SegmentState Param3T;
            typedef Param2T Param4T;
            typedef Param3T Param5T;

            inline ReturnType const& operator()(ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
                                                ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
                                                ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState)
            {
                return segmentId->second.segment.getInertia();
            };

            inline ReturnType const& operator()(ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
                                                ParameterTypeQualifier<Param2T>::RefToArgT p_jointState,
                                                ParameterTypeQualifier<Param3T>::RefToArgT p_segmentState, //current's state (current's body force)
                                                ParameterTypeQualifier<Param4T>::RefToArgT p_jointState2,
                                                ParameterTypeQualifier<Param5T>::RefToArgT p_segmentState2) //current's child's state (child's total force)
            {
                std::cout << "Inside project inertial operation 5 argument function call " << std::endl;
                return segmentId->second.segment.getInertia();
            };

    };

    template<>
    class project<kdl_tree_iterator, wrench>
    {
        public:

            enum
            {
                NumberOfParams = 5
            };

            typedef SegmentState ReturnType;
            typedef kdl_tree_iterator Param1T;
            typedef JointState Param2T;
            typedef SegmentState Param3T;
            typedef Param2T Param4T;
            typedef Param3T Param5T;

            inline ReturnType const& operator()(ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
                                                ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
                                                ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState)
            {
                //TODO: Place holder for future use
                return a_segmentState;
            };

            //this operation returns total force on the current body (current's updated state)

            inline ReturnType const& operator()(ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
                                                ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
                                                ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState, //current's old state (current's body force)
                                                ParameterTypeQualifier<Param4T>::RefToArgT p_jointState2,
                                                ParameterTypeQualifier<Param5T>::RefToConstT p_segmentState2) //current's child's state (child's total force)
            {

                a_segmentState = p_segmentState;

                #ifdef VERBOSE_CHECK_KDLE
                    std::cout << "Inside wrench operation Input Wrench of current element (current's body wrench) " << std::endl << p_segmentState.F << std::endl << std::endl;
                    std::cout << "Inside wrench operation Input Wrench of a child (child's total wrench) " << std::endl << p_segmentState2.F << std::endl << std::endl;
                #endif

                p_jointState2.torque = dot(p_segmentState2.Z, p_segmentState2.F);
                a_segmentState.F = p_segmentState.F + p_segmentState2.X * p_segmentState2.F;

                #ifdef VERBOSE_CHECK_KDLE
                    std::cout << "Inside wrench operation updated Wrench of current element " << std::endl << a_segmentState.F << std::endl << std::endl;
                    std::cout << "Inside wrench operation computed Torque of current joint " << std::endl << p_jointState2.torque << std::endl << std::endl;
                #endif
                return a_segmentState;
            };

        private:
            ReturnType a_segmentState;

    };

    //Direction here is the type name of enum
    template<>
    class DFSPolicy<KDL::Tree, outward>
    {
        public:

            DFSPolicy()=default;
           
            ~DFSPolicy()=default;

            template <typename OP>
            inline bool walk(typename ParameterTypeQualifier<KDL::Tree>::RefToConstT a_topology,
                             typename ParameterTypeQualifier<std::vector<typename OP::Param2T> >::RefToConstT a_jointStateVectorIn,
                             typename ParameterTypeQualifier<std::vector<typename OP::Param3T> >::RefToArgT a_linkStateVectorIn,
                             typename ParameterTypeQualifier<std::vector<typename OP::Param3T> >::RefToArgT a_linkStateVectorOut,
                             OP& a_op)
            {

                //this is forward/outward iterative walk. It works when there is no F_ext
                for (KDL::SegmentMap::const_iterator iter = a_topology.getSegments().begin(); iter != a_topology.getSegments().end(); ++iter)
                {
                    const KDL::TreeElement currentElement = iter->second;
                    #ifdef VERBOSE_CHECK_KDLE
                                std::cout << "Parent element name in current iteration " << currentElement.segment.getName() << std::endl;
                                std::cout << "State vector input " << a_linkStateVectorIn[currentElement.q_nr].segmentName << std::endl;
                    #endif
                    a_linkStateVectorOut[currentElement.q_nr] = a_linkStateVectorIn[currentElement.q_nr];

                    for (std::vector<KDL::SegmentMap::const_iterator>::const_iterator childIter = iter->second.children.begin(); childIter != iter->second.children.end(); childIter++)
                    {

                        #ifdef VERBOSE_CHECK_KDLE
                                        std::cout << "Current element name in current iteration " << (*childIter)->second.segment.getName() << std::endl;
                                        std::cout << "Current joint index and value " << (*childIter)->second.q_nr << " " << a_jointStateVectorIn[(*childIter)->second.q_nr].q << std::endl;
                        #endif
                        a_linkStateVectorIn[(*childIter)->second.q_nr] = a_op(*childIter, a_jointStateVectorIn[(*childIter)->second.q_nr], a_linkStateVectorOut[currentElement.q_nr]);
                    }
                }

                return true;
            };

            template <typename OP>
            inline bool walk(typename ParameterTypeQualifier<KDL::Tree>::RefToConstT a_topology,
                             typename ParameterTypeQualifier<std::vector<typename OP::Param2T> >::RefToConstT a_jointStateVectorIn,
                             typename ParameterTypeQualifier<std::vector<typename OP::Param2T> >::RefToArgT a_jointStateVectorOut, //introduce a separate mutable state representation, now is used for testing
                             typename ParameterTypeQualifier<std::vector<typename OP::Param3T> >::RefToConstT a_linkStateVectorIn,
                             typename ParameterTypeQualifier<std::vector<typename OP::Param3T> >::RefToArgT a_linkStateVectorOut,
                             OP& a_op)
            {
                SegmentState tempState;


                //this is forward/outward iterative walk
                for (KDL::SegmentMap::const_iterator iter = a_topology.getSegments().begin(); iter != a_topology.getSegments().end(); ++iter)
                {
                    const KDL::TreeElement currentElement = iter->second;
                    tempState = a_linkStateVectorIn[currentElement.q_nr];
                    #ifdef VERBOSE_CHECK_KDLE
                                std::cout << "Parent element name in current iteration " << currentElement.segment.getName() << std::endl;
                                std::cout << "State vector input " << a_linkStateVectorIn[currentElement.q_nr].segmentName << std::endl;
                                //            std::cout << "Parent Fext " << currentElement.q_nr << " " << a_linkStateVectorIn[currentElement.q_nr].Fext << std::endl;
                    #endif

                    for (std::vector<KDL::SegmentMap::const_iterator>::const_iterator childIter = iter->second.children.begin(); childIter != iter->second.children.end(); childIter++)
                    {

                        #ifdef VERBOSE_CHECK_KDLE
                                        std::cout << "Current element name in current iteration " << (*childIter)->second.segment.getName() << std::endl;
                                        std::cout << "Current joint index and value " << (*childIter)->second.q_nr << " " << a_jointStateVectorIn[(*childIter)->second.q_nr].q << std::endl;
                        #endif
                                        //TODO: Operation call needs to be corrected
                                        //               tempState = a_op(*childIter, a_jointStateVectorIn[(*childIter)->second.q_nr], tempState, a_jointStateVectorOut[(*childIter)->second.q_nr], a_linkStateVectorOut[(*childIter)->second.q_nr] );

                    }
                }
                return true;
            };

    };

    template<>
    class DFSPolicy<KDL::Tree, inward>
    {
        public:

            DFSPolicy()=default;
            ~DFSPolicy()=default;

            template <typename OP>
            inline static bool walk(typename ParameterTypeQualifier<KDL::Tree>::RefToConstT a_topology,
                                    typename ParameterTypeQualifier<std::vector<typename OP::Param2T> >::RefToConstT a_jointStateVectorIn,
                                    typename ParameterTypeQualifier<std::vector<typename OP::Param3T> >::RefToArgT a_linkStateVectorIn,
                                    typename ParameterTypeQualifier<std::vector<typename OP::Param3T> >::RefToArgT a_linkStateVectorOut,
                                    OP a_op)
            {
                return true;
            };

            template <typename OP>
            inline static bool walk(typename ParameterTypeQualifier<KDL::Tree>::RefToConstT a_topology,
                                    typename ParameterTypeQualifier<std::vector<typename OP::Param2T> >::RefToConstT a_jointStateVectorIn,
                                    typename ParameterTypeQualifier<std::vector<typename OP::Param2T> >::RefToArgT a_jointStateVectorOut, //introduce a separate mutable state representation, now is used for testing
                                    typename ParameterTypeQualifier<std::vector<typename OP::Param3T> >::RefToConstT a_linkStateVectorIn,
                                    typename ParameterTypeQualifier<std::vector<typename OP::Param3T> >::RefToArgT a_linkStateVectorOut,
                                    OP a_op)
            {

                SegmentState tempState;
                #ifdef VERBOSE_CHECK_KDLE
                    std::cout << std::endl << "This is reverse iteration/inward" << std::endl;
                #endif
                //this is reverse/inward iterative walk
                for (KDL::SegmentMap::const_reverse_iterator iter = a_topology.getSegments().rbegin(); iter != a_topology.getSegments().rend(); ++iter)
                {
                    const KDL::TreeElement currentElement = iter->second;
                    tempState = a_linkStateVectorIn[currentElement.q_nr];

                    #ifdef VERBOSE_CHECK_KDLE
                        std::cout << "Current element name in current reverse iteration " << currentElement.segment.getName() << std::endl;
                        std::cout << "Current joint index and value in reverse iteration " << currentElement.q_nr << " " << a_jointStateVectorIn[currentElement.q_nr].q << std::endl;
                        std::cout << "Spatial BODY force on a current element " << a_linkStateVectorIn[currentElement.q_nr].F << std::endl;
                    #endif

                    for (std::vector<KDL::SegmentMap::const_iterator>::const_iterator childIter = iter->second.children.begin(); childIter != iter->second.children.end(); childIter++)
                    {

                        #ifdef VERBOSE_CHECK_KDLE
                            std::cout << "Child element name in current  reverse iteration " << (*childIter)->second.segment.getName() << std::endl;
                            std::cout << "Child joint index and value " << (*childIter)->second.q_nr << " " << a_jointStateVectorIn[(*childIter)->second.q_nr].q << std::endl;
                            std::cout << "Total spatial force on a child " << a_linkStateVectorOut[(*childIter)->second.q_nr].F << std::endl;
                        #endif
//                        torques(j--)=dot(S[i],f[i]);
//                        f[i - 1] = f[i - 1] + X[i] * f[i];
//                        the second term in the 2nd expression should be summed for all children of the parent and then added to the parent's force (ID for trees).
//                        a_linkStateVectorIn[parentElement.q_nr].F = a_linkStateVectorIn[parentElement.q_nr].F + a_linkStateVectorIn[(*childIter)->second.q_nr].X * a_linkStateVectorIn[(*childIter)->second.q_nr].F;
//                        double torque = dot(a_linkStateVectorIn[(*childIter)->second.q_nr].Z, a_linkStateVectorIn[(*childIter)->second.q_nr].F);

                        tempState = a_op(*childIter, a_jointStateVectorIn[(*childIter)->second.q_nr], tempState, a_jointStateVectorOut[(*childIter)->second.q_nr], a_linkStateVectorOut[(*childIter)->second.q_nr]); //uses operations with 5 args
                        std::cout << "Torque on a current supporting joint " << (*childIter)->second.q_nr << " " << a_jointStateVectorOut[(*childIter)->second.q_nr].torque << std::endl << std::endl;
                    }
                    //TODO: this needs to be changed, string comparison is inefficient
                    if (iter->first != a_topology.getRootSegment()->first)
                        a_linkStateVectorOut[currentElement.q_nr] = tempState;

                    #ifdef VERBOSE_CHECK_KDLE
                        std::cout << "Total Spatial force on a current element " << a_linkStateVectorOut[currentElement.q_nr].F << std::endl;
                        std::cout << "Torque on a current supporting joint " << a_jointStateVectorOut[currentElement.q_nr].torque << std::endl << std::endl;
                    #endif
                }
                return true;
            };
    }; 

    
    typedef std::vector<Joint<grs::Pose<KDL::Vector, KDL::Rotation> > >::const_iterator grs_iterator;
    
    template<>
    class transform<grs_iterator, pose>
    {
        public:

            enum
            {
                NumberOfParams = 5
            };
            typedef grs_iterator Param1T;
            typedef ComputationalState<grs::Pose<KDL::Vector, KDL::Rotation>, grs::Twist<KDL::Vector, KDL::Vector>, StateSpaceType::CartesianSpace> ReturnType;
            typedef ComputationalState<grs::Pose<KDL::Vector, KDL::Rotation>, grs::Twist<KDL::Vector, KDL::Vector>, StateSpaceType::JointSpace >    Param2T;
            typedef ComputationalState<grs::Pose<KDL::Vector, KDL::Rotation>, grs::Twist<KDL::Vector, KDL::Vector>, StateSpaceType::CartesianSpace> Param3T;
            typedef Param2T Param4T;
            typedef Param3T Param5T;

            inline ReturnType const& operator()(ParameterTypeQualifier<Param1T>::RefToConstT jointId,
                                                ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
                                                ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState)
            {
                std::cout << " JOINT NAME " << jointId->getName() << std::endl<< std::endl;
                jointId->getPoseCurrentDistalToPredecessorDistal(p_jointState.q, a_segmentState.X);
                jointId->getPoseCurrentDistalToPredecessorRefJointFrame(p_jointState.q, a_segmentState.X);
                return a_segmentState;
            };

        private:
            ReturnType a_segmentState;

    };

    template<>
    class transform<grs_iterator, twist>
    {
        public:

            enum
            {
                NumberOfParams = 5
            };
            
            typedef ComputationalState<grs::Pose<KDL::Vector, KDL::Rotation>, grs::Twist<KDL::Vector, KDL::Vector>, StateSpaceType::CartesianSpace> ReturnType;
            typedef ComputationalState<grs::Pose<KDL::Vector, KDL::Rotation>, grs::Twist<KDL::Vector, KDL::Vector>, StateSpaceType::JointSpace > Param2T;
            typedef ComputationalState<grs::Pose<KDL::Vector, KDL::Rotation>, grs::Twist<KDL::Vector, KDL::Vector>, StateSpaceType::CartesianSpace> Param3T;
            typedef grs_iterator Param1T;
            typedef Param2T Param4T;
            typedef Param3T Param5T;

            inline ReturnType const& operator()(ParameterTypeQualifier<Param1T>::RefToConstT jointId,
                                                ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
                                                ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState)
                {
                    std::cout << " JOINT NAME " << jointId->getName() << std::endl<< std::endl;
                    jointId->getTwistCurrentDistalToPredecessorJointFrame(p_jointState.q, p_jointState.qdot, a_segmentState.Xdot);
//                    jointId->getTwistCurrentDistalToPredecessorDistal(p_jointState.q, p_jointState.qdot, a_segmentState.Xdot);
                    return a_segmentState;
                };


            inline ReturnType const& operator()(ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
                                                ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
                                                ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState, //current's state (current's body force)
                                                ParameterTypeQualifier<Param4T>::RefToArgT p_jointState2,
                                                ParameterTypeQualifier<Param5T>::RefToConstT p_segmentState2); //current's child's state (child's total force)

        private:
            ReturnType a_segmentState;
    };

    template<>
    class transform<grs_iterator, accTwist>
    {
        public:

            enum
            {
                NumberOfParams = 5
            };

            typedef ComputationalState<grs::Pose<KDL::Vector, KDL::Rotation>, grs::Twist<KDL::Vector, KDL::Vector>, StateSpaceType::CartesianSpace> ReturnType;
            typedef ComputationalState<grs::Pose<KDL::Vector, KDL::Rotation>, grs::Twist<KDL::Vector, KDL::Vector>, StateSpaceType::JointSpace > Param2T;
            typedef ComputationalState<grs::Pose<KDL::Vector, KDL::Rotation>, grs::Twist<KDL::Vector, KDL::Vector>, StateSpaceType::CartesianSpace> Param3T;
            typedef grs_iterator Param1T;
            typedef Param2T Param4T;
            typedef Param3T Param5T;

            inline ReturnType const& operator()( ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
                                                 ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
                                                 ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState)
            {

                return a_segmentState;
            };

            inline ReturnType const& operator()(ParameterTypeQualifier<Param1T>::RefToConstT segmentId,
                                                ParameterTypeQualifier<Param2T>::RefToConstT p_jointState,
                                                ParameterTypeQualifier<Param3T>::RefToConstT p_segmentState, //current's state (current's body force)
                                                ParameterTypeQualifier<Param4T>::RefToArgT p_jointState2,
                                                ParameterTypeQualifier<Param5T>::RefToConstT p_segmentState2); //current's child's state (child's total force)

        private:
            ReturnType a_segmentState;

    };

    
    /**
     * \brief Version of DFSPolicy with KinematicChain using geometric relations semantics and std::vector as container
     *
     * \param 
     */
    template<>
    class DFSPolicy< KinematicChain<grs::Pose<KDL::Vector, KDL::Rotation> > >
    {
        public:

            DFSPolicy()=default;
            ~DFSPolicy()=default;

            template <typename OP>
            inline static bool walk(KinematicChain<grs::Pose<KDL::Vector, KDL::Rotation> > const& a_topology,
                                    typename ParameterTypeQualifier<std::vector<typename OP::Param2T> >::RefToConstT a_jointStateVectorIn,
                                    typename ParameterTypeQualifier<std::vector<typename OP::Param3T> >::RefToArgT a_linkStateVectorIn,
                                    typename ParameterTypeQualifier<std::vector<typename OP::Param3T> >::RefToArgT a_linkStateVectorOut,
                                    OP a_op)


            {
                unsigned int i = 0;
                for (std::vector<Joint<grs::Pose<KDL::Vector, KDL::Rotation> > >::const_iterator iter = a_topology.jointsOfChain.begin(); iter != a_topology.jointsOfChain.end(); iter++)
                {
                    a_op(iter, a_jointStateVectorIn[i], a_linkStateVectorIn[i]);
                    i++;
                }
                return true;
            };

    };

};


#endif	/* FUNCTIONALCOMPUTATION_KDLTYPES_HPP */

