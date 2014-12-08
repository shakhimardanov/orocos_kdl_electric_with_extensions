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

/**
 * @file
 * @brief Chain specification
 */


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
    // Metadata of the specification language: dsl uuid
    static boost::uuids::uuid temp;
    static boost::uuids::name_generator uuid_generator(temp);
    static boost::uuids::uuid const metamodelID = uuid_generator("KDLE_DSL");
    
    // Metadata containg name and uuid of the language primitives (i.e. types)
    struct MetaData
    {
        // type instance name
        std::string instanceName;
        // type instance uuid
        boost::uuids::uuid instanceID;
    };
    
    /**
     * @brief A list of joint types. Includes 1-DoF (e.g REVOLUTE_X) or Multi-DoF (e.g SPHERICAL)
     *
     *
     */
    enum class JointTypes : int
    { 
        REVOLUTE_AXIS = 0,
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
    
    
    /**
     * @brief A tuple of joint properties that include, in the given order, type from JointTypes, inertia, clockwise and counterclockwise limits
     *
     * 
     */
    //should also include polarity of the joint, it defines forward and reverse relation between successor and predecessor segments
    typedef std::tuple<JointTypes, double, double, double> JointProperties;
    
    /**
     * @brief Link is a geometric primitive defined through a constraining geometric relation between two poses, base and tip. <br>
     * 
     * Its specification requires semantic and coordinate representation data  on link base and tip poses. <br>
     * Base pose is defined with respect to some world reference pose and tip pose is defined with respect to the base
     */
    template <typename PoseT>
    class Link
    {
        public:
           /**
            * @brief Default constructor auto generated by the compiler.
            */
            Link()=default;
            
           /**
            * @brief A constructor that link instance with the given name, base and tip frame specification.
            * 
            * @param givenName is a name of the link instance
            * @param rootF pose of base frame of the link. PoseT can be any pose type from geometric relations semantics (e.g. Pose<KDL::Frame>)
            * @param tipF pose of tip frame of the link. PoseT can be any pose type from geometric relations semantics (e.g. Pose<KDL::Vector, KDL::Rotation>)
            */
            Link(std::string const& givenName, typename ParameterTypeQualifier<PoseT>::RefToConstT rootF, typename ParameterTypeQualifier<PoseT>::RefToConstT tipF)
                : rootFrame_q0(rootF), tipFrame_q0(tipF)
            {
                classData.instanceName = givenName;
                classData.instanceID = uuid_generator(classData.instanceName);               
            };
            
            /**
        	 * @brief returns relative pose of link tip (distal) frame to link base (proximal) frame
             * 
             * @return relative pose between two frames of the link. PoseT can be any pose type from geometric relations semantics (e.g. Pose<KDL::Vector, KDL::Rotation>)
        	 */
            PoseT const& getPoseCurrentDistalToCurrentProximal() const;

            /**
        	 * @brief returns relative pose of link tip (distal) frame to world reference frame which is was used as construction time of the link.<br>
             * Here the world frame becomes some predecessors tip (distal) frame.
             * 
             * @return relative pose tip frame of the link wrt to the world/base frame. PoseT can be any pose type from geometric relations semantics (e.g. Pose<KDL::Vector, KDL::Rotation>)
        	 */
            PoseT const& getPoseCurrentDistalToPredecessorDistal() const;
            
            //return the name, uuid, and classuuid
            /**
             * @return name of the link instance
        	 */
            std::string const& getName() const {return classData.instanceName;};
            
            /**
             * @return UID of the link instance
        	 */
            boost::uuids::uuid const& getUID() const {return classData.instanceID;};
            
            /**
             * @return UID of the link class
        	 */
            boost::uuids::uuid const& getClassUID() const {return modelID;};
            
            /**
             * @brief Default destructor auto generated by the compiler.
             */
            ~Link()=default;
            
        private:
            PoseT rootFrame_q0;
            PoseT tipFrame_q0;
            mutable PoseT tipToProximalFrame_q0;
            mutable PoseT tipToDistalFrame_q0;
            
            struct MetaData classData;
            static boost::uuids::uuid const modelID;
            
        protected:
            bool getPoseCurrentDistalToCurrentProximalImpl(typename ParameterTypeQualifier<PoseT>::RefToArgT tip2rootpose) const;
            bool getPoseCurrentDistalToPredecessorDistalImpl(typename ParameterTypeQualifier<PoseT>::RefToArgT tip2tippose) const;
    };

    template <typename PoseT>
    boost::uuids::uuid const Link<PoseT>::modelID = uuid_generator("TypeNameLink");
    
    template <typename PoseT>
    PoseT const& Link<PoseT>::getPoseCurrentDistalToCurrentProximal() const 
    {
        if (!getPoseCurrentDistalToCurrentProximalImpl(tipToProximalFrame_q0))
            std::cout <<"Warning: can not return pose data " << std::endl;
        else
            return tipToProximalFrame_q0; 
    };
    
    template <typename PoseT>
    PoseT const& Link<PoseT>::getPoseCurrentDistalToPredecessorDistal() const 
    {
        if(!getPoseCurrentDistalToPredecessorDistalImpl(tipToDistalFrame_q0))
            std::cout <<"Warning: can not return pose data " << std::endl;
        else
            return tipToDistalFrame_q0; 
    };
    
    template <typename PoseT>
    bool Link<PoseT>::getPoseCurrentDistalToCurrentProximalImpl(typename ParameterTypeQualifier<PoseT>::RefToArgT  tip2rootpose) const
    {
        tip2rootpose = tipFrame_q0;
        return true; 
    };
    
    template <typename PoseT>
    bool Link<PoseT>::getPoseCurrentDistalToPredecessorDistalImpl(typename ParameterTypeQualifier<PoseT>::RefToArgT tip2tippose) const
    {
        tip2tippose = grs::compose(rootFrame_q0,tipFrame_q0);
        return true;
    };

    /**
     * @brief A list of the frame types that can be attached to a segment.<br>
     * Each frame type is unique and allows attachment of a particular geometric or dynamic primitives to a segment. <br>
     * For instances a joint connecting two segments can only be specified by two joint attachment frames on these segments. <br>
     * Another example, an inertia attachment frame type specifies w.r.t which frame an inertia of the rigid body is given.
     */
    enum class FrameType : char
    {
        JOINT='J',
        INERTIA='I',
        SHAPE='S',
        BODY='B',
    };
    
    /**
     * @brief Redirects FrameType value to standard output
     *
     */
    inline std::ostream& operator<<(std::ostream &out, FrameType tag )
    {
        return out << static_cast<char>(tag);
    };
    
    
    /**
     * @brief
     *
     */
    template <typename PoseT>
    class AttachmentFrame
    {
        public:
            /**
             * @brief
             *
             */
            AttachmentFrame()=default;
            
            /**
             * @brief
             *
             */
            AttachmentFrame(typename ParameterTypeQualifier<PoseT>::RefToConstT pose, typename ParameterTypeQualifier<FrameType>::RefToConstT type):poseData(pose), tagData(type)
            {
                std::string combinedName = poseData.getPoint().getName() + poseData.getOrientationFrame().getName() + poseData.getBody().getName() +
                                           poseData.getRefPoint().getName() + poseData.getRefOrientationFrame().getName() + poseData.getRefBody().getName() + poseData.getCoordinateFrame().getName();
                instanceID = uuid_generator(combinedName);
            };
            
            /**
             * @brief
             *
             */
            boost::uuids::uuid const& getUID() const {return instanceID;};
            
            /**
             * @brief
             *
             */
            PoseT poseData;
            
            /**
             * @brief
             *
             */
            FrameType tagData;
        private:
            boost::uuids::uuid instanceID;
    };
    
    /**
     * @brief
     *
     */
    template <typename PoseT>
    AttachmentFrame<PoseT> createAttachmentFrame(PoseT const& pose, FrameType const& type)
    {
        return AttachmentFrame<PoseT>(pose, type);
    };
    
    /**
     * @brief  A segment specification requires existing link instance and a list of attachment frames
     *
     */
    template <typename PoseT>
    class Segment
    {
        public:
            /**
             * @brief
             *
             */
            Segment()=default;
            
            /**
             * @brief
             *
             */
            Segment(std::string const& givenName, typename ParameterTypeQualifier< Link<PoseT> >::RefToConstT link, std::vector< AttachmentFrame<PoseT> > const& listOfFrames) : linkOfSegment(link), attachmentFrames(listOfFrames)
            {
                classData.instanceName = givenName;
                classData.instanceID = uuid_generator(classData.instanceName);
            };
            
            /**
        	 * @brief add attachment frames for joints
        	 *
        	 * @param a list of joint attachment frame poses
        	 */
            std::vector< AttachmentFrame<PoseT> > const& getAttachmentFrames() const
            {  
                return attachmentFrames;
            };

            /**
        	 * @brief add attachment frames for joints
        	 *
        	 * @param a list of joint attachment frame poses
        	 */
            Link<PoseT> const& getSegmentLink() const
            {
                return linkOfSegment;
            };            
            
            //return the name, uuid, and classuuid
            /**
             * @brief
             *
             */
            std::string const& getName() const {return classData.instanceName;};
            
            /**
             * @brief
             *
             */
            boost::uuids::uuid const& getUID() const {return classData.instanceID;};
            
            /**
             * @brief
             *
             */
            boost::uuids::uuid const& getClassUID() const {return modelID;};
            
            /**
             * @brief
             *
             */
            ~Segment(){};
        private:
            struct MetaData classData;
            static boost::uuids::uuid const modelID;
            Link<PoseT>  linkOfSegment;
            std::vector< AttachmentFrame<PoseT> > attachmentFrames;
    };
    
    template <typename PoseT >
    boost::uuids::uuid const Segment<PoseT>::modelID = uuid_generator("TypeNameSegment");
    
    /**
     * @brief constraints on how to qualify joint attachment frames
     *
     */
    enum class SemanticConstraint : char
    {
        CoincidentJointFrames = 'C', // this indicates that joint frames initially  coincide
        SameReferenceFrame = 'S', //this indicates requirements on additional segment info
    };
    
    /**
     * @brief A joint specification requires two segment instances and pose information of joint attachment frames on each of these segments
     *
     */
    template <typename PoseT>
    class Joint
    {
        public:
            /**
             * @brief
             *
             */
            Joint()=default;
            
            /**
             * @brief
             *
             */
            Joint(std::string const& givenName, AttachmentFrame<PoseT> const& segmentJointFrame, Segment<PoseT> const& firstSegmentID, 
                    AttachmentFrame<PoseT> const& refSegmentJointFrame, Segment<PoseT> const& refSegmentID, JointProperties const& properties )
                    :successorFrame(segmentJointFrame), predecessorFrame(refSegmentJointFrame), jointprops(properties), targetSegment(&firstSegmentID), refSegment(&refSegmentID), isNotValid(false)
            {
                classData.instanceName = givenName;
                classData.instanceID = uuid_generator(classData.instanceName);
                if(!isValid())
                {
                    isNotValid = true;
                }
            };
            
            /**
        	 * @brief compute and return relative pose of the tip of the given segment with respect to its predecessor's tip frame
        	 *
        	 * @param tipFramePose
        	 */
            void getPoseCurrentDistalToPredecessorDistal(std::vector<double> const& jointvalues, typename ParameterTypeQualifier<PoseT>::RefToArgT tipFramePose) const;
            
            /**
             * @brief
             *
             */
            void getPoseCurrentXToPredecessorRefJointFrame(std::vector<double> const& jointvalues, typename ParameterTypeQualifier<PoseT>::RefToArgT tipFramePose) const;
            
            /**
             * @brief
             *
             */
            void getPoseCurrentXToPredecessorRefJointFrame(std::vector<double> const& jointvalues, AttachmentFrame<PoseT> const& segmentFrame, typename ParameterTypeQualifier<PoseT>::RefToArgT tipFramePose) const;
            
            /**
        	 * @brief compute and return relative pose between two joint frames of two segments: current/successor joint frame and previous/predecessor joint frame
        	 *
        	 * @param jointFramePose is a vector of doubles for multiDoF, for 1 DoF it is of size 1
        	 */
            void getPoseOfJointFrames(std::vector<double> const& jointvalues, typename ParameterTypeQualifier<PoseT>::RefToArgT relativePose) const;
           
            /**
        	 * @brief compute and return relative twist between two segment link frames: current tip and previous tip frames
        	 *
        	 * @param jointtwistvalues is a vector of doubles for multiDoF, for 1 DoF it is of size 1
        	 */
            template <typename TwistT>
            void getTwistCurrentXToPredecessorJointFrame(std::vector<double> const& jointvalues, std::vector<double> const& jointtwistvalues, TwistT &relativeTwist) const;
            
            /**
        	 * @brief compute and return relative twist between two segment link frames: current tip and previous tip frames
        	 *
        	 * @param jointtwistvalues is a vector of doubles for multiDoF, for 1 DoF it is of size 1
        	 */
            template <typename TwistT>
            void getTwistCurrentXToPredecessorJointFrame(std::vector<double> const& jointvalues, std::vector<double> const& jointtwistvalues, AttachmentFrame<PoseT> const& segmentFrame, TwistT &relativeTwist) const;
            
            /**
        	 * @brief compute and return relative twist between to joint frames of two segments: current root and previous tip frames
        	 *
        	 * @param jointFramePose is a vector of doubles for multiDoF, for 1 DoF it is of size 1
        	 */
            template <typename TwistT>
            void getTwistOfJointFrames(std::vector<double> const& jointtwistvalues, TwistT &relativeTwist) const;
            
            /**
             * @brief return type of the joint, e.g 1 DoF rotational around Z axis
             * @return
             */
            JointTypes const& getType() const {return std::get<0>(jointprops); };
            
            /**
             * @brief return string name of the type of the joint, e.g "REVOLUTE_Z" for 1 DoF rotational around Z axis
             * @return
             */
            std::string const getTypeName() const 
            {
                switch (std::get<0>(jointprops)) 
                {
                    case JointTypes::REVOLUTE_AXIS:
                        return "REVOLUTE_AXIS";
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
                    case JointTypes::HELICAL:
                        return "HELICAL";
                    case JointTypes::FREEMOTION:
                        return "FREEMOTION";
                    case JointTypes::CYLINDRICAL:
                        return "CYLINDRICAL";
                    case JointTypes::PLANAR:
                        return "PLANAR";
                    case JointTypes::SPHERICAL:
                        return "SPHERICAL";
                    default:
                        return "NONE";
                }
            };
            
            /**
             * @brief
             *
             */
            std::vector<Segment<PoseT> > const& getSegments() const{ };
            
            /**
             * @brief
             *
             */
            AttachmentFrame<PoseT> const& getRefJointFrame() const{ return predecessorFrame; };
            
            /**
             * @brief
             *
             */
            AttachmentFrame<PoseT> const& getJointFrame() const{ return successorFrame; };
            
            //return the name, uuid, and classuuid
            /**
             * @brief
             *
             */
            std::string const& getName() const {return classData.instanceName;};
            
            /**
             * @brief
             *
             */
            boost::uuids::uuid const& getUID() const {return classData.instanceID;};
            
            /**
             * @brief
             *
             */
            boost::uuids::uuid const& getClassUID() const {return modelID;};
            
            /**
             * @brief
             *
             */
            ~Joint(){};
            
        private:
            AttachmentFrame<PoseT>  successorFrame;
            AttachmentFrame<PoseT>  predecessorFrame;
            struct MetaData classData;
            static boost::uuids::uuid const modelID;
            JointProperties jointprops;
            Segment<PoseT> const*  targetSegment;
            Segment<PoseT> const*  refSegment;
            bool isNotValid;
            
        protected:
            /**
             * @brief
             *
             */
            bool isValid() const;
            
            /**
             * @brief
             *
             */
            bool getPoseOfJointFramesImpl(std::vector<double> const& jointvalues, typename ParameterTypeQualifier<PoseT>::RefToArgT posetochange) const;
            
            /**
             * @brief
             *
             */
            bool getPoseDistalToDistalImpl(std::vector<double> const& jointvalues, typename ParameterTypeQualifier<PoseT>::RefToArgT tipFramePose) const;
            
            /**
             * @brief
             *
             */
            bool getPoseCurrentXToPredecessorRefJointFrameImpl(std::vector<double> const& jointvalues, typename ParameterTypeQualifier<PoseT>::RefToArgT tipFramePose) const;
            
            /**
             * @brief
             *
             */
            template <typename TwistT>
            bool getTwistCurrentXToPredecessorJointFrameImpl(std::vector<double> const& jointvalues, std::vector<double> const& jointtwistvalues, TwistT &relativeTwist) const;
            
            /**
             * @brief
             *
             */
            template <typename TwistT>
            bool getTwistOfJointFramesImpl(std::vector<double> const& jointtwistvalues, TwistT &relativeTwist) const;
    };
    
    template <typename PoseT>
    boost::uuids::uuid const Joint<PoseT>::modelID = uuid_generator("TypeNameJoint");
    
    template <typename PoseT>
    bool Joint<PoseT>::isValid() const
    {
        if( successorFrame.poseData.getBody() != predecessorFrame.poseData.getBody() )
        {
            if( !((successorFrame.tagData == FrameType::JOINT) && (predecessorFrame.tagData == FrameType::JOINT)) )
                return false;
            else 
                return true;
        }
        else
            return false;
    }
    
    template <typename PoseT>
    void Joint<PoseT>::getPoseCurrentDistalToPredecessorDistal(std::vector<double> const& jointvalues, typename ParameterTypeQualifier<PoseT>::RefToArgT tipFramePose) const
    {
        if(!getPoseDistalToDistalImpl(jointvalues, tipFramePose))
            std::cout <<"Warning: can not return pose data. Check whether the joint is correctly constructed " << std::endl;
        else
            std::cout << "Inside Joint DistalToDistal Pose Impl " << std::endl << tipFramePose <<std::endl;
        return;
    }
    
    template <typename PoseT>
    void Joint<PoseT>::getPoseCurrentXToPredecessorRefJointFrame(std::vector<double> const& jointvalues, typename ParameterTypeQualifier<PoseT>::RefToArgT tipFramePose) const
    {
        if(!getPoseCurrentXToPredecessorRefJointFrameImpl(jointvalues, tipFramePose))
            std::cout <<"Warning: can not return pose data. Check whether the joint is correctly constructed " << std::endl;
        else
            std::cout << "Inside Distal to Predecessor Joint Pose Impl " << std::endl << tipFramePose <<std::endl;
        return;
    }
    
    template <typename PoseT>
    void Joint<PoseT>::getPoseOfJointFrames(std::vector<double> const& jointvalues, typename ParameterTypeQualifier<PoseT>::RefToArgT relativePose) const
    {
        if (!getPoseOfJointFramesImpl(jointvalues, relativePose))
            std::cout <<"Warning: can not return pose data. Check whether the joint is correctly constructed " << std::endl;
        else
            std::cout << "Inside Joint Pose Impl " << std::endl << relativePose <<std::endl;
        return;
    
    }
    
    template <typename PoseT>
        template <typename TwistT>
    void Joint<PoseT>::getTwistCurrentXToPredecessorJointFrame(std::vector<double> const& jointvalues, std::vector<double> const& jointtwistvalues, TwistT &relativeTwist) const
    {
        if(!getTwistCurrentXToPredecessorJointFrameImpl(jointvalues, jointtwistvalues, relativeTwist))
            std::cout <<"Warning: can not return twist data. Check whether the joint is correctly constructed " << std::endl;
        else
            std::cout << "Inside Joint DistalToRefJoint Twist Impl " << std::endl << relativeTwist <<std::endl;
        return;
    }
            
    template <typename PoseT>
      template <typename TwistT>
    void Joint<PoseT>::getTwistOfJointFrames(std::vector<double> const& jointtwistvalues, TwistT& relativeTwist) const
    {
        if(!getTwistOfJointFramesImpl(jointtwistvalues, relativeTwist))
            std::cout <<"Warning: can not return twist data. Check whether the joint is correctly constructed " << std::endl;
        else
            std::cout << "Inside Joint Twist Impl " << std::endl << relativeTwist <<std::endl;
        return;
    }
    
    //Specialization for two argument pose with KDL coordinate representation
    template <> inline 
    bool Joint< grs::Pose<KDL::Vector, KDL::Rotation> >::getPoseOfJointFramesImpl(std::vector<double> const& jointvalues, grs::Pose<KDL::Vector, KDL::Rotation> &posetochange) const
    {       
        //Constraint: Joint frame origins coincide
        //create position and orientation using semantics of provided ref and target joint frames
        if(isNotValid)
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
                 case JointTypes::REVOLUTE_AXIS:
                {
                    break;
                }
                
                case JointTypes::PRISMATIC_X:
                {
                    break;
                }
                
                case JointTypes::PRISMATIC_Y:
                {
                    break;
                }
                
                case JointTypes::PRISMATIC_Z:
                {
                    break;
                }
                
                case JointTypes::PRISMATIC_AXIS:
                {
                    break;
                }
                
                case JointTypes::CYLINDRICAL:
                {
                    break;
                }
                case JointTypes::FREEMOTION:
                {
                    break;
                }
                case JointTypes::HELICAL:
                {
                    break;
                }
                case JointTypes::PLANAR:
                {
                    break;
                }
                case JointTypes::SPHERICAL:
                {
                    break;
                }
                case JointTypes::NONE:
                {
                    break;
                }
            }
            return true;
        }
            
        
    }
    
    //Specialization for two argument pose with KDL coordinate representation
    template<> inline
    bool Joint< grs::Pose<KDL::Vector, KDL::Rotation> >::getPoseDistalToDistalImpl(std::vector<double> const& jointvalues, grs::Pose<KDL::Vector, KDL::Rotation> &tipFramePose) const
    {
        
        //target joint to ref joint
        if(!getPoseOfJointFramesImpl(jointvalues, tipFramePose))
        {
            return false;
        }
        else
        {
            //target distal to ref distal
            tipFramePose = grs::compose(grs::compose(predecessorFrame.poseData,refSegment->getSegmentLink().getPoseCurrentDistalToCurrentProximal().inverse2()),
                           grs::compose(tipFramePose,grs::compose(targetSegment->getSegmentLink().getPoseCurrentDistalToCurrentProximal(),successorFrame.poseData.inverse2())));
            return true;
        }
    }
  
    //Specialization for two argument pose with KDL coordinate representation
    template<> inline
    bool Joint< grs::Pose<KDL::Vector, KDL::Rotation> >::getPoseCurrentXToPredecessorRefJointFrameImpl(std::vector<double> const &jointvalues, grs::Pose<KDL::Vector, KDL::Rotation> &posetochange) const
    {
        //target joint to ref joint
        if (!getPoseOfJointFramesImpl(jointvalues, posetochange))
        {
            return false;
        }
        else
        {
            //distal to target joint
            grs::Pose<KDL::Vector, KDL::Rotation> tempSegmentTipToJointFramePose = grs::compose(targetSegment->getSegmentLink().getPoseCurrentDistalToCurrentProximal(),successorFrame.poseData.inverse2());
            //distal to ref joint
            posetochange = grs::compose(posetochange,tempSegmentTipToJointFramePose);
            return true;
        }
    }
    
    //Specialization for two argument vector with KDL coordinate representation
    template <> 
        template <> inline
    bool Joint< grs::Pose<KDL::Vector, KDL::Rotation> >::getTwistOfJointFramesImpl(std::vector<double> const& jointtwistvalues, grs::Twist<KDL::Vector, KDL::Vector> &relativeTwist) const
    {
        if(isNotValid)
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
                
                case JointTypes::REVOLUTE_AXIS:
                {
                    break;
                }
                
                case JointTypes::PRISMATIC_X:
                {
                    break;
                }
                
                case JointTypes::PRISMATIC_Y:
                {
                    break;
                }
                
                case JointTypes::PRISMATIC_Z:
                {
                    break;
                }
                
                case JointTypes::PRISMATIC_AXIS:
                {
                    break;
                }
                
                case JointTypes::CYLINDRICAL:
                {
                    break;
                }
                case JointTypes::FREEMOTION:
                {
                    break;
                }
                case JointTypes::HELICAL:
                {
                    break;
                }
                case JointTypes::PLANAR:
                {
                    break;
                }
                case JointTypes::SPHERICAL:
                {
                    break;
                }
                case JointTypes::NONE:
                {
                    break;
                }
            }
            return true;
        }
    }
    
    //Specialization for two argument vector with KDL coordinate representation
    template <> 
        template <> inline
    bool Joint< grs::Pose<KDL::Vector, KDL::Rotation> >::getTwistCurrentXToPredecessorJointFrameImpl(std::vector<double> const& jointvalues, std::vector<double> const& jointtwistvalues, grs::Twist<KDL::Vector, KDL::Vector> &relativeTwist) const
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
            grs::Position<KDL::Vector> tempSegmentJointToTipFramePosition = grs::compose(targetSegment->getSegmentLink().getPoseCurrentDistalToCurrentProximal(),successorFrame.poseData.inverse2()).getPosition< KDL::Vector >().inverse();
            
            //target joint to refjoint orientation
            grs::Pose<KDL::Vector, KDL::Rotation> tempSegmentJointToRootFramePose;
            getPoseOfJointFramesImpl(jointvalues, tempSegmentJointToRootFramePose);
            grs::Orientation<KDL::Rotation> tempJointToRefJointOrientation = tempSegmentJointToRootFramePose.getOrientation<KDL::Rotation>();
            
            //distal to refjoint orientation
            grs::Pose<KDL::Vector, KDL::Rotation> tempSegmentDistalToRefJointFramePose;
            getPoseCurrentXToPredecessorRefJointFrameImpl(jointvalues, tempSegmentDistalToRefJointFramePose);
            grs::Orientation<KDL::Rotation> tempDistalToRefJointOrientation = tempSegmentDistalToRefJointFramePose.getOrientation<KDL::Rotation>();
            //put distal to targetjoint vector into refjoint coordinate frame
            if(tempSegmentJointToTipFramePosition.changeCoordinateFrame(tempJointToRefJointOrientation))
            {
                //change its reference point
                relativeTwist.changePointBody(tempSegmentJointToTipFramePosition);
                //express it in distal's coordinate frame 
                relativeTwist.changeCoordinateFrame(tempDistalToRefJointOrientation.inverse2());
            }
            return true;
        }
    }
    
 
   
    /**
     * @brief  A kinematic chain specification requires a sequence of  joint instances
     *
     */
    template <typename PoseT, typename ContainerT = std::vector< Joint<PoseT> > >
    class KinematicChain
    {
        public:
            typedef typename ContainerT::const_iterator IteratorT;
            
        public:
            /**
             * @brief
             *
             */
            KinematicChain()=default;
            
            /**
             * @brief
             *
             */
            KinematicChain(std::string const& givenName, ContainerT const& jointlist)
            {
                jointsOfChain = jointlist;
                classData.instanceName = givenName;
                classData.instanceID = uuid_generator(classData.instanceName);
                isNotValid = false;
                if(!isValid())
                {
                    isNotValid = true;
                }
            };
            
            /**
             * @brief
             * @return
             */
            IteratorT addJoint(Joint<PoseT> const& newjoint);
            
            /**
             * @brief
             * @return
             */
            IteratorT getJoint(std::string const& jointname) const;
            
            /**
             * @brief
             * @return
             */
            unsigned int getNrOfJoints();
            
            /**
             * @brief
             * @return
             */
            unsigned int getNrOfSegments();
            
            //return the name, uuid, and classuuid
            /**
             * @brief
             *
             */
            std::string const& getName() const {return classData.instanceName;};
            
            /**
             * @brief
             *
             */
            boost::uuids::uuid const& getUID() const {return classData.instanceID;};
            
            /**
             * @brief
             *
             */
            boost::uuids::uuid const& getClassUID() const {return modelID;};
            
            /**
             * @brief
             *
             */
            ~KinematicChain(){};
    
            ContainerT jointsOfChain;
        private:
            struct MetaData classData;
            static boost::uuids::uuid const modelID;
            bool isNotValid;
        
        protected:   
            
            /**
             * @brief  return true if the constructed chain is valid
             *
             */
            bool isValid() const;
    };
    
    template <typename PoseT, typename ContainerT>
    boost::uuids::uuid const KinematicChain<PoseT, ContainerT >::modelID = uuid_generator("TypeNameKinematicChain");
    
    template <typename PoseT, typename ContainerT>
    bool KinematicChain<PoseT, ContainerT >::isValid() const
    {
        //put all joint frame uuids in a single array
        std::vector<boost::uuids::uuid> listOfJointFrameUUIDs;
        for(typename KinematicChain<PoseT>::IteratorT iter=jointsOfChain.begin(); iter!=jointsOfChain.end(); iter++)
        {
            listOfJointFrameUUIDs.push_back(iter->getJointFrame().getUID());
            listOfJointFrameUUIDs.push_back(iter->getRefJointFrame().getUID());
        }
        //sort the array
        std::sort(listOfJointFrameUUIDs.begin(), listOfJointFrameUUIDs.end());
        unsigned int originalSize = listOfJointFrameUUIDs.size();
        //find the consecutive duplicates in the sorted array and erase them
        listOfJointFrameUUIDs.erase(std::unique(listOfJointFrameUUIDs.begin(), listOfJointFrameUUIDs.end()), listOfJointFrameUUIDs.end());
        unsigned int newSize = listOfJointFrameUUIDs.size();
        //if the new array size is different than the original then some joint frames were reused in joint construction 
        if (originalSize != newSize)
            return false;
        else
            return true;
    };
    
    
    template <typename PoseT, typename ContainerT>
    typename KinematicChain<PoseT, ContainerT >::IteratorT KinematicChain<PoseT, ContainerT >::addJoint(Joint<PoseT> const& newjoint)
    {
        if(isNotValid)
            std::cout << "Warning: Existing chain is not valid " << std::endl;
        else
            return jointsOfChain.insert(jointsOfChain.end(), newjoint);
    }
    
    
    
    template <typename PoseT, typename ContainerT>
    typename KinematicChain<PoseT, ContainerT >::IteratorT KinematicChain<PoseT, ContainerT >::getJoint(std::string const& jointname) const
    {
        if(isNotValid)
            std::cout << "Warning: Existing chain is not valid " << std::endl;
        else
        {
            for(typename KinematicChain<PoseT>::IteratorT iter=jointsOfChain.begin(); iter!=jointsOfChain.end(); iter++)
            {
               if(iter->getName() == jointname)
                return iter;
            }
            std::cout << "Warning: Joint with such a name does not exist " << std::endl;
        }
        
    }
    
    template <typename PoseT, typename ContainerT>
    unsigned int KinematicChain<PoseT, ContainerT >::getNrOfJoints()
    {
        if(isNotValid)
            std::cout << "Warning: Existing chain is not valid " << std::endl;
        else
            return jointsOfChain.size();
    }
    
    template <typename PoseT, typename ContainerT>
    unsigned int KinematicChain<PoseT, ContainerT >::getNrOfSegments()
    {
        if(isNotValid)
            std::cout << "Warning: Existing chain is not valid " << std::endl;
        else
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
