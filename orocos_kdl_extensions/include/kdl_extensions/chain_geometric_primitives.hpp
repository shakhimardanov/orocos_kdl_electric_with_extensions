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

#ifndef _CHAIN_GEOMETRIC_PRIMITIVES_HPP_
#define _CHAIN_GEOMETRIC_PRIMITIVES_HPP_

#include <cstdlib>
#include <list>
#include <vector>
#include <tuple>
#include <algorithm>
#include <functional>
#include <string>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <kdl_extensions/geometric_semantics_kdl.hpp>
#include <kdl_extensions/functionalcomputation_kdl.hpp>

namespace kdle
{
    //should also include polarity of the joint, it defines forward and reverse relation between successor and predecessor segments
    typedef std::tuple<KDL::Joint::JointType, double, double, double> JointProperties;
    
    // Metadata of the language
    static boost::uuids::uuid temp;
    static boost::uuids::name_generator uuid_generator(temp);
    static boost::uuids::uuid const metamodelID = uuid_generator("KDLE_DSL");
    
    //Metadata containg name and uuid of the primitive
    struct MetaData
    {
        std::string instanceName;
        boost::uuids::uuid instanceID;
   
    };
    
    /**
     * \brief Link specification requires pose information on link base and tip frames
     *
     * 
     */
    template <typename PoseT>
    class Link
    {
        public:
            Link()=default;
            Link(const std::string &givenName, typename ParameterTypeQualifier<PoseT>::RefToConstT rootF, typename ParameterTypeQualifier<PoseT>::RefToConstT tipF)
                : rootFrame_q0(rootF), tipFrame_q0(tipF)
            {
                classData.instanceName = givenName;
                classData.instanceID = uuid_generator(classData.instanceName);               
            };
            
            /**
        	 * \brief
        	 */
            PoseT const& getCurrentTipToCurrentRootPose();

            /**
        	 * \brief
        	 */
            PoseT const& getCurrentTipToPredecessorTipPose();
            
            //return the name, uuid, and classuuid
            std::string const& getName() const {return classData.instanceName;};
            boost::uuids::uuid const& getUID() const {return classData.instanceID;};
            boost::uuids::uuid const& getClassUID() const {return modelID;};
            ~Link(){};
            
        private:
            PoseT rootFrame_q0;
            PoseT tipFrame_q0;
            PoseT tipToRootFrame_q0;
            PoseT tipToTipFrame_q0;
            
            struct MetaData classData;
            static boost::uuids::uuid modelID;
            
        protected:
            bool getCurrentTipToCurrentRootPoseImpl(PoseT &tip2rootpose);
            bool getCurrentTipToPredecessorTipPoseImpl(PoseT &tip2tippose);
    };

    template <typename PoseT>
    boost::uuids::uuid Link<PoseT>::modelID = uuid_generator("TypeNameLink");
    
    template <typename PoseT>
    PoseT const& Link<PoseT>::getCurrentTipToCurrentRootPose()  
    {
        if (!getCurrentTipToCurrentRootPoseImpl(tipToRootFrame_q0))
            std::cout <<"Warning: can not return pose data " << std::endl;
        else
            return tipToRootFrame_q0; 
    };
    
    template <typename PoseT>
    PoseT const& Link<PoseT>::getCurrentTipToPredecessorTipPose()  
    {
        if(!getCurrentTipToPredecessorTipPoseImpl(tipToTipFrame_q0))
            std::cout <<"Warning: can not return pose data " << std::endl;
        else
            return tipToTipFrame_q0; 
    };
    
    template <typename PoseT>
    bool Link<PoseT>::getCurrentTipToCurrentRootPoseImpl(PoseT &tip2rootpose)
    {
        tip2rootpose = tipFrame_q0;
        return true; 
    };
    
    template <typename PoseT>
    bool Link<PoseT>::getCurrentTipToPredecessorTipPoseImpl(PoseT &tip2tippose)
    {
        tip2tippose = grs::compose(rootFrame_q0,tipFrame_q0);
        return true;
    };
    
    /**
     * \brief Segment specification requires existing link instance and a list of attachment frames
     *
     */
    enum class FrameType : char
    {
        JOINT='J',
        INERTIA='I',
        SHAPE='S',
        BODY='B',
    };
    
    std::ostream& operator<<(std::ostream &out, FrameType tag )
    {
        return out << static_cast<char>(tag);
    };
    
    
    /**
     * \brief Segment specification requires existing link instance and a list of attachment frames
     *
     */
    template <typename PoseT>
    class AttachmentFrame
    {
        public:
            AttachmentFrame()=default;
            AttachmentFrame(PoseT const& pose, FrameType const& type):poseData(pose), tagData(type){};
            PoseT poseData;
            FrameType tagData;
    };
    
    /**
     * \brief Segment specification requires existing link instance and a list of attachment frames
     *
     */
    template <typename PoseT>
    AttachmentFrame<PoseT> createAttachmentFrame(PoseT const& pose, FrameType const& type)
    {
        return AttachmentFrame<PoseT>(pose, type);
    };
    
    /**
     * \brief Segment specification requires existing link instance and a list of attachment frames
     *
     */
    template <typename PoseT>
    class Segment
    {
        public:
            Segment()=default;
            Segment(const std::string &givenName, typename ParameterTypeQualifier< Link<PoseT> >::RefToConstT link, std::vector< AttachmentFrame<PoseT> > const& listOfFrames)
            {
                classData.instanceName = givenName;
                classData.instanceID = uuid_generator(classData.instanceName);
                attachmentFrames = listOfFrames;
                linkOfSegment = link;
            };
            
            /**
        	 * \brief add attachment frames for joints
        	 *
        	 * \param a list of joint attachment frame poses
        	 */
            std::vector< AttachmentFrame<PoseT> > const& getAttachmentFrames()
            {  
                return attachmentFrames;
            };

            /**
        	 * \brief add attachment frames for joints
        	 *
        	 * \param a list of joint attachment frame poses
        	 */
            Link<PoseT> & getSegmentLink()
            {
                return linkOfSegment;
            };            
            
            //return the name, uuid, and classuuid
            std::string const& getName() const {return classData.instanceName;};
            boost::uuids::uuid const& getUID() const {return classData.instanceID;};
            boost::uuids::uuid const& getClassUID() const {return modelID;};
            
            ~Segment(){};
        private:
            std::vector< AttachmentFrame<PoseT> > attachmentFrames;
            Link<PoseT>  linkOfSegment;
            struct MetaData classData;
            static boost::uuids::uuid modelID;
    };
    
    template <typename PoseT >
    boost::uuids::uuid Segment<PoseT>::modelID = uuid_generator("TypeNameSegment");
    
    /**
     * \brief constraints on how to qualify joint attachment frames
     *
     */
    enum class SemanticConstraint : char
    {
        CoincidentJointFrames = 'C', // this indicates that joint frames initially  coincide
        SameReferenceFrame = 'S', //this indicates requirements on additional segment info
    };
    
    /**
     * \brief Joint specification requires two segment instances and pose information of joint attachment frames on each of these segments
     *
     */
    template <typename PoseT>
    class Joint
    {
        public:
            Joint()=default;
            Joint(const std::string &givenName, AttachmentFrame<PoseT> const& segmentJointFrame, Segment<PoseT> & firstSegmentID, 
                    AttachmentFrame<PoseT> const& refSegmentJointFrame, Segment<PoseT> & refSegmentID, JointProperties const& properties )
                    :successorFrame(segmentJointFrame), predecessorFrame(refSegmentJointFrame), jointprops(properties), targetSegment(&firstSegmentID), refSegment(&refSegmentID)
            {
                classData.instanceName = givenName;
                classData.instanceID = uuid_generator(classData.instanceName);
            };
            
            /**
        	 * \brief compute and return relative pose of the tip of the given segment with respect to its predecessor's tip frame
        	 *
        	 * \param tipFramePose
        	 */
            void getCurrentTipToPredecessorTipPose(std::vector<double> const& jointvalues, typename ParameterTypeQualifier<PoseT>::RefToArgT tipFramePose);
            
            /**
        	 * \brief compute and return relative pose between two joint frames of two segments: current/successor joint frame and previous/predecessor joint frame
        	 *
        	 * \param jointFramePose is a vector of doubles for multiDoF, for 1 DoF it is of size 1
        	 */
            void getPoseOfJointFrames(std::vector<double> const& jointvalues, typename ParameterTypeQualifier<PoseT>::RefToArgT relativePose);
           
            /**
        	 * \brief compute and return relative twist between two segment link frames: current tip and previous tip frames
        	 *
        	 * \param jointtwistvalues is a vector of doubles for multiDoF, for 1 DoF it is of size 1
        	 */
            template <typename TwistT>
            void getCurrentTipToPredecessorTipTwist(std::vector<double> const& jointtwistvalues, TwistT &relativeTwist);
            
            /**
        	 * \brief compute and return relative twist between to joint frames of two segments: current root and previous tip frames
        	 *
        	 * \param jointFramePose is a vector of doubles for multiDoF, for 1 DoF it is of size 1
        	 */
            template <typename TwistT>
            void getCurrentRootToPredecessorTipTwist(std::vector<double> const& jointtwistvalues, TwistT &relativeTwist);
            
            /**
             * \brief return type of the joint, e.g 1 DoF rotational around Z axis
             *
             */
            KDL::Joint::JointType const& getType() const {return std::get<0>(jointprops); };
            
            /**
             * \brief return string name of the type of the joint, e.g "RotZ" for 1 DoF rotational around Z axis
             *
             */
            std::string const getTypeName() const 
            {
                switch (std::get<0>(jointprops)) 
                {
                    case KDL::Joint::JointType::RotAxis:
                        return "RotAxis";
                    case KDL::Joint::JointType::TransAxis:
                        return "TransAxis";
                    case KDL::Joint::JointType::RotX:
                        return "RotX";
                    case KDL::Joint::JointType::RotY:
                        return "RotY";
                    case KDL::Joint::JointType::RotZ:
                        return "RotZ";
                    case KDL::Joint::JointType::TransX:
                        return "TransX";
                    case KDL::Joint::JointType::TransY:
                        return "TransY";
                    case KDL::Joint::JointType::TransZ:
                        return "TransZ";
                    case KDL::Joint::JointType::None:
                        return "None";
                    default:
                        return "None";
                }
            };
            std::vector<Segment<PoseT> > const& getSegments() const{};
            
            //return the name, uuid, and classuuid
            std::string const& getName() const {return classData.instanceName;};
            boost::uuids::uuid const& getUID() const {return classData.instanceID;};
            boost::uuids::uuid const& getClassUID() const {return modelID;};
            ~Joint(){};
            
        private:
            struct MetaData classData;
            static boost::uuids::uuid modelID;
            AttachmentFrame<PoseT> successorFrame;
            AttachmentFrame<PoseT> predecessorFrame;
            JointProperties jointprops;
            Segment<PoseT>  *targetSegment;
            Segment<PoseT>  *refSegment;
            
        protected:
            bool getPoseOfJointFramesImpl(std::vector<double> const& jointvalues, typename ParameterTypeQualifier<PoseT>::RefToArgT posetochange);
            bool getTipToTipPoseImpl(std::vector<double> const& jointvalues, typename ParameterTypeQualifier<PoseT>::RefToArgT tipFramePose);
            template <typename TwistT>
            bool getTipToTipTwistImpl(std::vector<double> const& jointtwistvalues, TwistT &relativeTwist){};
            template <typename TwistT>
            bool getRootToTipTwistImpl(std::vector<double> const& jointtwistvalues, TwistT &relativeTwist){};
    };
    
    template <typename PoseT>
    boost::uuids::uuid Joint<PoseT>::modelID = uuid_generator("TypeNameJoint");
    
    template <typename PoseT>
    void Joint<PoseT>::getCurrentTipToPredecessorTipPose(std::vector<double> const& jointvalues, typename ParameterTypeQualifier<PoseT>::RefToArgT tipFramePose)
    {
        if(!getTipToTipPoseImpl(jointvalues, tipFramePose))
            std::cout <<"Warning: can not return pose data " << std::endl;
        else
            std::cout << "Inside Joint TipToTip Pose Impl " << tipFramePose <<std::endl;
        return;
    }
    
    template <typename PoseT>
    void Joint<PoseT>::getPoseOfJointFrames(std::vector<double> const& jointvalues, typename ParameterTypeQualifier<PoseT>::RefToArgT relativePose)
    {
        if (!getPoseOfJointFramesImpl(jointvalues, relativePose))
            std::cout <<"Warning: can not return pose data " << std::endl;
        else
            std::cout << "Inside Joint Pose Impl " << relativePose <<std::endl;
        return;
    
    }
    
    template <typename PoseT>
        template <typename TwistT>
    void Joint<PoseT>::getCurrentTipToPredecessorTipTwist(std::vector<double> const& jointtwistvalues, TwistT &relativeTwist)
    {
        if(!getTipToTipTwistImpl(jointtwistvalues, relativeTwist))
            std::cout <<"Warning: can not return twist data " << std::endl;
        else
            std::cout << "Inside Joint TipToTip Twist Impl " << relativeTwist <<std::endl;
        return;
    }
            
    template <typename PoseT>
      template <typename TwistT>
    void Joint<PoseT>::getCurrentRootToPredecessorTipTwist(std::vector<double> const& jointtwistvalues, TwistT& relativeTwist)
    {
        if(!getRootToTipTwistImpl(jointtwistvalues, relativeTwist))
            std::cout <<"Warning: can not return twist data " << std::endl;
        else
            std::cout << "Inside Joint TipToTip Twist Impl " << relativeTwist <<std::endl;
        return;
    }
    
    //Specialization for two argument pose with KDL coordinate representation
    template <>
    bool Joint< grs::Pose<KDL::Vector, KDL::Rotation> >::getPoseOfJointFramesImpl(std::vector<double> const& jointvalues, grs::Pose<KDL::Vector, KDL::Rotation> &posetochange)
    {
        //refJoint Frame w.r.t. some global ref B
        grs::Pose<KDL::Vector, KDL::Rotation> tempSegmentRootToTipFramePose = refSegment->getSegmentLink().getCurrentTipToCurrentRootPose().inverse2();
//        std::cout << "tempSegmentRootToTipFramePose " << tempSegmentRootToTipFramePose << std::endl; 
        grs::Pose<KDL::Vector, KDL::Rotation> tempSegmentTipToTipFramePose = refSegment->getSegmentLink().getCurrentTipToPredecessorTipPose();
//        std::cout << "tempSegmentTipToTipFramePose " << tempSegmentTipToTipFramePose << std::endl;
        grs::Pose<KDL::Vector, KDL::Rotation> tempSegmentJointToRootFramePose = grs::compose(grs::compose(tempSegmentTipToTipFramePose,tempSegmentRootToTipFramePose), predecessorFrame.poseData);
//        std::cout << "tempSegmentJointToRootFramePose " << tempSegmentJointToRootFramePose << std::endl;
        //targetJoint Frame w.r.t. some ref B
        tempSegmentRootToTipFramePose = targetSegment->getSegmentLink().getCurrentTipToCurrentRootPose().inverse2();
//        std::cout << "tempSegmentRootToTipFramePose " << tempSegmentRootToTipFramePose << std::endl; 
        tempSegmentTipToTipFramePose = targetSegment->getSegmentLink().getCurrentTipToPredecessorTipPose();
//        std::cout << "tempSegmentTipToTipFramePose " << tempSegmentTipToTipFramePose << std::endl;
        grs::Pose<KDL::Vector, KDL::Rotation> tempSegmentJointToRootFramePose2 =grs::compose(grs::compose(tempSegmentTipToTipFramePose,tempSegmentRootToTipFramePose), successorFrame.poseData);
//        std::cout << "tempSegmentJointToRootFramePose " << tempSegmentJointToRootFramePose2  << std::endl;
        //targetJoint Frame w.r.t. refJoint Frame
        posetochange = grs::compose(tempSegmentJointToRootFramePose2,tempSegmentJointToRootFramePose.inverse2());
//        std::cout << "posetochange " <<  posetochange   << std::endl;
        grs::Orientation<KDL::Rotation> newOrientation = grs::Orientation<KDL::Rotation>(posetochange.getOrientation<KDL::Rotation>().getSemantics(), KDL::Rotation::RotZ(jointvalues[0]));
//        std::cout <<   newOrientation   << std::endl;
//        posetochange.changeCoordinateFrame(grs::Orientation<KDL::Rotation>(posetochange.getOrientation<KDL::Rotation>().getSemantics(), KDL::Rotation::RotZ(jointvalues[0])));
        
       
        
        return true;
    }
    
    //Specialization for two argument pose with KDL coordinate representation
    template<>
    bool Joint< grs::Pose<KDL::Vector, KDL::Rotation> >::getTipToTipPoseImpl(std::vector<double> const& jointvalues, grs::Pose<KDL::Vector, KDL::Rotation> &tipFramePose)
    {
        
        return true;
    }
 /*   
    //Specialization for two argument vector with KDL coordinate representation
    template <>
        template <>
    bool Joint< grs::Pose<KDL::Vector, KDL::Rotation> >::getTipToTipTwistImpl(std::vector<double> const& jointtwistvalues, grs::Twist<KDL::Vector, KDL::Vector> &relativeTwist)
    {
        return true;
    }
    
    //Specialization for two argument vector with KDL coordinate representation
    template <>
        template <>
    bool Joint< grs::Pose<KDL::Vector, KDL::Rotation> >::getRootToTipTwistImpl(std::vector<double> const& jointtwistvalues, grs::Twist<KDL::Vector, KDL::Vector> &relativeTwist)
    {
        return true;
    }
*/    
    /**
     * \brief  KinematicChain specification requires a sequence of  joint instances
     *
     */
    template <typename PoseT, typename ContainerT = std::vector< Joint<PoseT> > >
    class KinematicChain
    {
            typedef typename ContainerT::const_iterator IteratorT;
        public:
            KinematicChain()=default;
            KinematicChain(std::string const& givenName, ContainerT const& jointlist)
            {
                jointsOfChain = jointlist;
                classData.instanceName = givenName;
                classData.instanceID = uuid_generator(classData.instanceName);
            };
            
            bool addJoint(Joint<PoseT> const& newjoint);
            
            IteratorT getJoint(std::string const& jointname) const;
            
            unsigned int getNrOfJoints();
            
            unsigned int getNrOfSegments();
            
            //return the name, uuid, and classuuid
            std::string const& getName() const {return classData.instanceName;};
            boost::uuids::uuid const& getUID() const {return classData.instanceID;};
            boost::uuids::uuid const& getClassUID() const {return modelID;};
            
            ~KinematicChain(){};
    
            ContainerT jointsOfChain;
        private:
            struct MetaData classData;
            static boost::uuids::uuid modelID;
        
        protected:
            
            /**
             * \brief  return true if the constructed chain is valid
             *
             */
            bool isKinematicChainValid();
    };
    
    template <typename PoseT, typename ContainerT>
    boost::uuids::uuid KinematicChain<PoseT, ContainerT >::modelID = uuid_generator("TypeNameKinematicChain");
    
    template <typename PoseT, typename ContainerT>
    bool KinematicChain<PoseT, ContainerT >::isKinematicChainValid()
    { 
        return true;
    };
    
    
    template <typename PoseT, typename ContainerT>
    bool KinematicChain<PoseT, ContainerT >::addJoint(Joint<PoseT> const& newjoint)
    {
        return true;
    }
    
    template <typename PoseT, typename ContainerT>
    typename KinematicChain<PoseT, ContainerT >::IteratorT KinematicChain<PoseT, ContainerT >::getJoint(std::string const& jointname) const
    {
        
    }
    
    template <typename PoseT, typename ContainerT>
    unsigned int KinematicChain<PoseT, ContainerT >::getNrOfJoints()
    {
    
    }
    
    template <typename PoseT, typename ContainerT>
    unsigned int KinematicChain<PoseT, ContainerT >::getNrOfSegments()
    {
    
    }
    
    //friction, inertia, scale/ratio, offset, damping, elasticity
    typedef std::tuple<double, double, double> TransmissionProperties;
    
    /**
     * \brief 
     *
     * \param 
     */
    template<typename JointT>
    void make_transmission(JointT const& joint_in, TransmissionProperties const& transprops)
    {
    
    };

    /**
     * \brief 
     *
     * \param 
     */
    template<typename PoseT, typename InertiaTensorT>
    void make_rigidbody(typename ParameterTypeQualifier< Segment<PoseT> >::RefToConstT segment_in, 
                        typename ParameterTypeQualifier<InertiaTensorT>::RefToConstT inertiaTensor, 
                        typename ParameterTypeQualifier<PoseT>::RefToConstT inertiaFrame )
    {
    
    };

}


#endif //~_CHAIN_GEOMETRIC_PRIMITIVES_HPP_
