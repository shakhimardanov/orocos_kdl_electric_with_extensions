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

#include <vector>
#include <tuple>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <kdl_extensions/functionalcomputation_util.hpp>
#include <kdl_extensions/geometric_semantics_kdl.hpp>


namespace kdle
{
    enum class JointTypes : int
    { 
        REVOLUTE_AXI = 0,
        REVOLUTE_X,
        REVOLUTE_Y,
        REVOLUTE_Z,
        PRISMATIC_AXIS,
        PRISMATIC_X,
        PRISMATIC_Y,
        PRISMATIC_Z,
        HELICAL,
        CYLINDRICAL,
        PLANAR,
        SPHERICAL,
        FREEMOTION,
        NONE,
    };
    
    //should also include polarity of the joint, it defines forward and reverse relation between successor and predecessor segments
    typedef std::tuple<JointTypes, double, double, double> JointProperties;
    
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
            PoseT const& getCurrentDistalToCurrentProximalPose() const;

            /**
        	 * \brief
        	 */
            PoseT const& getCurrentDistalToPredecessorDistalPose() const;
            
            //return the name, uuid, and classuuid
            std::string const& getName() const {return classData.instanceName;};
            boost::uuids::uuid const& getUID() const {return classData.instanceID;};
            boost::uuids::uuid const& getClassUID() const {return modelID;};
            ~Link(){};
            
        private:
            PoseT rootFrame_q0;
            PoseT tipFrame_q0;
            mutable PoseT tipToProximalFrame_q0;
            mutable PoseT tipToDistalFrame_q0;
            
            struct MetaData classData;
            static boost::uuids::uuid modelID;
            
        protected:
            bool getCurrentDistalToCurrentProximalPoseImpl(typename ParameterTypeQualifier<PoseT>::RefToArgT tip2rootpose) const;
            bool getCurrentDistalToPredecessorDistalPoseImpl(typename ParameterTypeQualifier<PoseT>::RefToArgT tip2tippose) const;
    };

    template <typename PoseT>
    boost::uuids::uuid Link<PoseT>::modelID = uuid_generator("TypeNameLink");
    
    template <typename PoseT>
    PoseT const& Link<PoseT>::getCurrentDistalToCurrentProximalPose() const 
    {
        if (!getCurrentDistalToCurrentProximalPoseImpl(tipToProximalFrame_q0))
            std::cout <<"Warning: can not return pose data " << std::endl;
        else
            return tipToProximalFrame_q0; 
    };
    
    template <typename PoseT>
    PoseT const& Link<PoseT>::getCurrentDistalToPredecessorDistalPose() const 
    {
        if(!getCurrentDistalToPredecessorDistalPoseImpl(tipToDistalFrame_q0))
            std::cout <<"Warning: can not return pose data " << std::endl;
        else
            return tipToDistalFrame_q0; 
    };
    
    template <typename PoseT>
    bool Link<PoseT>::getCurrentDistalToCurrentProximalPoseImpl(typename ParameterTypeQualifier<PoseT>::RefToArgT  tip2rootpose) const
    {
        tip2rootpose = tipFrame_q0;
        return true; 
    };
    
    template <typename PoseT>
    bool Link<PoseT>::getCurrentDistalToPredecessorDistalPoseImpl(typename ParameterTypeQualifier<PoseT>::RefToArgT tip2tippose) const
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
    
    inline std::ostream& operator<<(std::ostream &out, FrameType tag )
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
            AttachmentFrame(typename ParameterTypeQualifier<PoseT>::RefToConstT pose, typename ParameterTypeQualifier<FrameType>::RefToConstT type):poseData(pose), tagData(type){};
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
            Segment(std::string const& givenName, typename ParameterTypeQualifier< Link<PoseT> >::RefToConstT link, std::vector< AttachmentFrame<PoseT> > const& listOfFrames) : linkOfSegment(link), attachmentFrames(listOfFrames)
            {
                classData.instanceName = givenName;
                classData.instanceID = uuid_generator(classData.instanceName);
            };
            
            /**
        	 * \brief add attachment frames for joints
        	 *
        	 * \param a list of joint attachment frame poses
        	 */
            std::vector< AttachmentFrame<PoseT> > const& getAttachmentFrames() const
            {  
                return attachmentFrames;
            };

            /**
        	 * \brief add attachment frames for joints
        	 *
        	 * \param a list of joint attachment frame poses
        	 */
            Link<PoseT> const& getSegmentLink() const
            {
                return linkOfSegment;
            };            
            
            //return the name, uuid, and classuuid
            std::string const& getName() const {return classData.instanceName;};
            boost::uuids::uuid const& getUID() const {return classData.instanceID;};
            boost::uuids::uuid const& getClassUID() const {return modelID;};
            
            ~Segment(){};
        private:
            struct MetaData classData;
            static boost::uuids::uuid modelID;
            Link<PoseT>  linkOfSegment;
            std::vector< AttachmentFrame<PoseT> > attachmentFrames;
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
            Joint(std::string const& givenName, AttachmentFrame<PoseT> const& segmentJointFrame, Segment<PoseT> const& firstSegmentID, 
                    AttachmentFrame<PoseT> const& refSegmentJointFrame, Segment<PoseT> const& refSegmentID, JointProperties const& properties )
                    :successorFrame(segmentJointFrame), predecessorFrame(refSegmentJointFrame), jointprops(properties), targetSegment(firstSegmentID), refSegment(refSegmentID)
            {
                classData.instanceName = givenName;
                classData.instanceID = uuid_generator(classData.instanceName);
            };
            
            /**
        	 * \brief compute and return relative pose of the tip of the given segment with respect to its predecessor's tip frame
        	 *
        	 * \param tipFramePose
        	 */
            void getCurrentDistalToPredecessorDistalPose(std::vector<double> const& jointvalues, typename ParameterTypeQualifier<PoseT>::RefToArgT tipFramePose);
            
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
            void getCurrentDistalToPredecessorJointTwist(std::vector<double> const& jointvalues, std::vector<double> const& jointtwistvalues, TwistT &relativeTwist);
            
            /**
        	 * \brief compute and return relative twist between to joint frames of two segments: current root and previous tip frames
        	 *
        	 * \param jointFramePose is a vector of doubles for multiDoF, for 1 DoF it is of size 1
        	 */
            template <typename TwistT>
            void getTwistOfJointFrames(std::vector<double> const& jointtwistvalues, TwistT &relativeTwist);
            
            /**
             * \brief return type of the joint, e.g 1 DoF rotational around Z axis
             *
             */
            JointTypes const& getType() const {return std::get<0>(jointprops); };
            
            /**
             * \brief return string name of the type of the joint, e.g "REVOLUTE_Z" for 1 DoF rotational around Z axis
             *
             */
            std::string const getTypeName() const 
            {
                switch (std::get<0>(jointprops)) 
                {
                    case JointTypes::REVOLUTE_AXI:
                        return "REVOLUTE_AXI";
                    case JointTypes::PRISMATIC_AXIS:
                        return "PRISMATIC_AXIS";
                    case JointTypes::REVOLUTE_X:
                        return "REVOLUTE_X";
                    case JointTypes::REVOLUTE_Y:
                        return "REVOLUTE_Y";
                    case JointTypes::REVOLUTE_Z:
                        return "REVOLUTE_Z";
                    case JointTypes::PRISMATIC_X:
                        return "PRISMATIC_X";
                    case JointTypes::PRISMATIC_Y:
                        return "PRISMATIC_Y";
                    case JointTypes::PRISMATIC_Z:
                        return "PRISMATIC_Z";
                    case JointTypes::NONE:
                        return "NONE";
                    default:
                        return "NONE";
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
            AttachmentFrame<PoseT>  successorFrame;
            AttachmentFrame<PoseT>  predecessorFrame;
            JointProperties jointprops;
            Segment<PoseT>  targetSegment;
            Segment<PoseT>  refSegment;
            
        protected:
            bool getPoseOfJointFramesImpl(std::vector<double> const& jointvalues, typename ParameterTypeQualifier<PoseT>::RefToArgT posetochange);
            bool getDistalToDistalPoseImpl(std::vector<double> const& jointvalues, typename ParameterTypeQualifier<PoseT>::RefToArgT tipFramePose);
            template <typename TwistT>
            bool getCurrentDistalToPredecessorJointTwistImpl(std::vector<double> const& jointvalues, std::vector<double> const& jointtwistvalues, TwistT &relativeTwist);
            template <typename TwistT>
            bool getTwistOfJointFramesImpl(std::vector<double> const& jointtwistvalues, TwistT &relativeTwist);
    };
    
    template <typename PoseT>
    boost::uuids::uuid Joint<PoseT>::modelID = uuid_generator("TypeNameJoint");
    
    template <typename PoseT>
    void Joint<PoseT>::getCurrentDistalToPredecessorDistalPose(std::vector<double> const& jointvalues, typename ParameterTypeQualifier<PoseT>::RefToArgT tipFramePose)
    {
        if(!getDistalToDistalPoseImpl(jointvalues, tipFramePose))
            std::cout <<"Warning: can not return pose data " << std::endl;
        else
            std::cout << "Inside Joint DistalToDistal Pose Impl " << std::endl << tipFramePose <<std::endl;
        return;
    }
    
    template <typename PoseT>
    void Joint<PoseT>::getPoseOfJointFrames(std::vector<double> const& jointvalues, typename ParameterTypeQualifier<PoseT>::RefToArgT relativePose)
    {
        if (!getPoseOfJointFramesImpl(jointvalues, relativePose))
            std::cout <<"Warning: can not return pose data " << std::endl;
        else
            std::cout << "Inside Joint Pose Impl " << std::endl << relativePose <<std::endl;
        return;
    
    }
    
    template <typename PoseT>
        template <typename TwistT>
    void Joint<PoseT>::getCurrentDistalToPredecessorJointTwist(std::vector<double> const& jointvalues, std::vector<double> const& jointtwistvalues, TwistT &relativeTwist)
    {
        if(!getCurrentDistalToPredecessorJointTwistImpl(jointvalues, jointtwistvalues, relativeTwist))
            std::cout <<"Warning: can not return twist data " << std::endl;
        else
            std::cout << "Inside Joint DistalToDistal Twist Impl " << std::endl << relativeTwist <<std::endl;
        return;
    }
            
    template <typename PoseT>
      template <typename TwistT>
    void Joint<PoseT>::getTwistOfJointFrames(std::vector<double> const& jointtwistvalues, TwistT& relativeTwist)
    {
        if(!getTwistOfJointFramesImpl(jointtwistvalues, relativeTwist))
            std::cout <<"Warning: can not return twist data " << std::endl;
        else
            std::cout << "Inside Joint Twist Impl " << std::endl << relativeTwist <<std::endl;
        return;
    }
    
    //Specialization for two argument pose with KDL coordinate representation
    template <> inline 
    bool Joint< grs::Pose<KDL::Vector, KDL::Rotation> >::getPoseOfJointFramesImpl(std::vector<double> const& jointvalues, grs::Pose<KDL::Vector, KDL::Rotation> &posetochange)
    {       
        //Constraint: Joint frame origins coincide
        //create position and orientation using semantics of provided ref and target joint frames
        if(!((successorFrame.tagData == FrameType::JOINT) && (predecessorFrame.tagData == FrameType::JOINT)))
        {
            return false;
        }
        else
        {
            switch(this->getType())
            {
                case JointTypes::REVOLUTE_Z:
                {
                    grs::Position<KDL::Vector> originPosition(successorFrame.poseData.getPoint(), successorFrame.poseData.getBody(), 
                                                              predecessorFrame.poseData.getPoint(), predecessorFrame.poseData.getBody(), 
                                                              predecessorFrame.poseData.getOrientationFrame(), KDL::Vector(0,0,0));
                    grs::Orientation<KDL::Rotation> frameRelOrientation(successorFrame.poseData.getOrientationFrame(), successorFrame.poseData.getBody(), 
                                                                        predecessorFrame.poseData.getOrientationFrame(), predecessorFrame.poseData.getBody(), 
                                                                        predecessorFrame.poseData.getOrientationFrame(), KDL::Rotation::RotZ(jointvalues[0]));
                    posetochange = grs::Pose<KDL::Vector, KDL::Rotation> (originPosition,frameRelOrientation);
                    break;
                }

                case JointTypes::REVOLUTE_X:
                {
                    grs::Position<KDL::Vector> originPosition(successorFrame.poseData.getPoint(), successorFrame.poseData.getBody(), 
                                                              predecessorFrame.poseData.getPoint(), predecessorFrame.poseData.getBody(), 
                                                              predecessorFrame.poseData.getOrientationFrame(), KDL::Vector(0,0,0));
                    grs::Orientation<KDL::Rotation> frameRelOrientation(successorFrame.poseData.getOrientationFrame(), successorFrame.poseData.getBody(), 
                                                                        predecessorFrame.poseData.getOrientationFrame(), predecessorFrame.poseData.getBody(), 
                                                                        predecessorFrame.poseData.getOrientationFrame(), KDL::Rotation::RotX(jointvalues[0]));
                    posetochange = grs::Pose<KDL::Vector, KDL::Rotation> (originPosition,frameRelOrientation);
                    break;
                }

                case JointTypes::REVOLUTE_Y:
                {
                    grs::Position<KDL::Vector> originPosition(successorFrame.poseData.getPoint(), successorFrame.poseData.getBody(), 
                                                              predecessorFrame.poseData.getPoint(), predecessorFrame.poseData.getBody(), 
                                                              predecessorFrame.poseData.getOrientationFrame(), KDL::Vector(0,0,0));
                    grs::Orientation<KDL::Rotation> frameRelOrientation(successorFrame.poseData.getOrientationFrame(), successorFrame.poseData.getBody(), 
                                                                        predecessorFrame.poseData.getOrientationFrame(), predecessorFrame.poseData.getBody(), 
                                                                        predecessorFrame.poseData.getOrientationFrame(), KDL::Rotation::RotY(jointvalues[0]));
                    posetochange = grs::Pose<KDL::Vector, KDL::Rotation> (originPosition,frameRelOrientation);
                    break;
                }
            }
            return true;
        }
            
        
    }
    
    //Specialization for two argument pose with KDL coordinate representation
    template<> inline
    bool Joint< grs::Pose<KDL::Vector, KDL::Rotation> >::getDistalToDistalPoseImpl(std::vector<double> const& jointvalues, grs::Pose<KDL::Vector, KDL::Rotation> &tipFramePose)
    {
        
        //target joint to ref joint
        if(!getPoseOfJointFramesImpl(jointvalues, tipFramePose))
        {
            return false;
        }
        else
        {
            //target distal to ref distal
            tipFramePose = grs::compose(grs::compose(predecessorFrame.poseData,refSegment.getSegmentLink().getCurrentDistalToCurrentProximalPose().inverse2()),
                           grs::compose(tipFramePose,grs::compose(targetSegment.getSegmentLink().getCurrentDistalToCurrentProximalPose(),successorFrame.poseData.inverse2())));
            return true;
        }
    }
  
    //Specialization for two argument vector with KDL coordinate representation
    template <> 
        template <> inline
    bool Joint< grs::Pose<KDL::Vector, KDL::Rotation> >::getTwistOfJointFramesImpl(std::vector<double> const& jointtwistvalues, grs::Twist<KDL::Vector, KDL::Vector> &relativeTwist)
    {
         if(!((successorFrame.tagData == FrameType::JOINT) && (predecessorFrame.tagData == FrameType::JOINT)))
        {
            return false;
        }
        else
        {
            switch(getType())
            {
                case JointTypes::REVOLUTE_Z:
                {
                    grs::LinearVelocity<KDL::Vector> originLinearVel(successorFrame.poseData.getPoint(), successorFrame.poseData.getBody(), 
                                                                     predecessorFrame.poseData.getBody(), predecessorFrame.poseData.getOrientationFrame(), KDL::Vector(0,0,0));
                    grs::AngularVelocity<KDL::Vector> frameAngularVel(successorFrame.poseData.getBody(), predecessorFrame.poseData.getBody(), 
                                                                      predecessorFrame.poseData.getOrientationFrame(), KDL::Vector(0, 0, jointtwistvalues[0]));
                    relativeTwist = grs::Twist<KDL::Vector, KDL::Vector> (originLinearVel,frameAngularVel);
                    break;
                }

                case JointTypes::REVOLUTE_X:
                {
                    grs::LinearVelocity<KDL::Vector> originLinearVel(successorFrame.poseData.getPoint(), successorFrame.poseData.getBody(), 
                                                                     predecessorFrame.poseData.getBody(), predecessorFrame.poseData.getOrientationFrame(), KDL::Vector(0,0,0));
                    grs::AngularVelocity<KDL::Vector> frameAngularVel(successorFrame.poseData.getBody(), predecessorFrame.poseData.getBody(), 
                                                                      predecessorFrame.poseData.getOrientationFrame(), KDL::Vector(jointtwistvalues[0], 0, 0));
                    relativeTwist = grs::Twist<KDL::Vector, KDL::Vector> (originLinearVel,frameAngularVel);
                    break;
                }

                case JointTypes::REVOLUTE_Y:
                {
                    grs::LinearVelocity<KDL::Vector> originLinearVel(successorFrame.poseData.getPoint(), successorFrame.poseData.getBody(), 
                                                                     predecessorFrame.poseData.getBody(), predecessorFrame.poseData.getOrientationFrame(), KDL::Vector(0,0,0));
                    grs::AngularVelocity<KDL::Vector> frameAngularVel(successorFrame.poseData.getBody(), predecessorFrame.poseData.getBody(), 
                                                                      predecessorFrame.poseData.getOrientationFrame(), KDL::Vector(0, jointtwistvalues[0], 0));
                    relativeTwist = grs::Twist<KDL::Vector, KDL::Vector> (originLinearVel,frameAngularVel);
                    break;
                }
            }
            return true;
        }
    }
    
    //Specialization for two argument vector with KDL coordinate representation
    template <> 
        template <> inline
    bool Joint< grs::Pose<KDL::Vector, KDL::Rotation> >::getCurrentDistalToPredecessorJointTwistImpl(std::vector<double> const& jointvalues, std::vector<double> const& jointtwistvalues, grs::Twist<KDL::Vector, KDL::Vector> &relativeTwist)
    {
        if(!getTwistOfJointFramesImpl(jointtwistvalues, relativeTwist))
        {
            return false;
        }
        else
        {
            //distal to proximal: targetSegment->getSegmentLink().getCurrentDistalToCurrentProximalPose();
            //joint to proximal inverse: successorFrame.poseData.inverse2();
            //distal to target joint: grs::compose(targetSegment->getSegmentLink().getCurrentDistalToCurrentProximalPose(),successorFrame.poseData.inverse2());
            grs::Position<KDL::Vector> tempSegmentJointToTipFramePosition = grs::compose(targetSegment.getSegmentLink().getCurrentDistalToCurrentProximalPose(),successorFrame.poseData.inverse2()).getPosition< KDL::Vector >().inverse();
            //target joint to ref joint
            grs::Pose<KDL::Vector, KDL::Rotation> tempSegmentJointToRootFramePose;
            getPoseOfJointFramesImpl(jointvalues, tempSegmentJointToRootFramePose);
            
            grs::Orientation<KDL::Rotation> tempJointToRefJointOrientation = tempSegmentJointToRootFramePose.getOrientation<KDL::Rotation>();
            //put distalToTarget joint vector into ref joint coordinate frame
            if(tempSegmentJointToTipFramePosition.changeCoordinateFrame(tempJointToRefJointOrientation))
            //change its reference point
                    relativeTwist.changePointBody(tempSegmentJointToTipFramePosition);
            return true;
        }
    }
    
 
   
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
            
            IteratorT addJoint(Joint<PoseT> const& newjoint);
            
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
    typename KinematicChain<PoseT, ContainerT >::IteratorT KinematicChain<PoseT, ContainerT >::addJoint(Joint<PoseT> const& newjoint)
    {
        
        return jointsOfChain.insert(jointsOfChain.end(), newjoint);
    }
    
    
    
    template <typename PoseT, typename ContainerT>
    typename KinematicChain<PoseT, ContainerT >::IteratorT KinematicChain<PoseT, ContainerT >::getJoint(std::string const& jointname) const
    {
        
    }
    
    template <typename PoseT, typename ContainerT>
    unsigned int KinematicChain<PoseT, ContainerT >::getNrOfJoints()
    {
        return jointsOfChain.size();
    }
    
    template <typename PoseT, typename ContainerT>
    unsigned int KinematicChain<PoseT, ContainerT >::getNrOfSegments()
    {
        return jointsOfChain.size();
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
