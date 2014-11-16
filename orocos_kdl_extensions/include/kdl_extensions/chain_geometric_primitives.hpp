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
#include <kdl_extensions/functionalcomputation_kdltypes.hpp>

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
            Link(const std::string &givenName, typename ParameterTypeQualifier<PoseT>::RefToConstT rootF, typename ParameterTypeQualifier<PoseT>::RefToConstT tipF)
            {
                classData.instanceName = givenName;
                classData.instanceID = uuid_generator(classData.instanceName);
                rootFrame_q0 = rootF;
                tipFrame_q0 = tipF;
               
            };
            
            /**
        	 * \brief
        	 */
            PoseT const& getTipToRootPose();
            
            //return the name, uuid, and classuuid
            std::string const& getName() const {return classData.instanceName;};
            boost::uuids::uuid const& getUID() const {return classData.instanceID;};
            boost::uuids::uuid const& getClassUID() const {return modelID;};
            ~Link(){};
        private:
            PoseT rootFrame_q0;
            PoseT tipFrame_q0;
            PoseT tipToRootFrame_q0;
            
            struct MetaData classData;
            static boost::uuids::uuid modelID;
    };

    template <typename PoseT>
    boost::uuids::uuid Link<PoseT>::modelID = uuid_generator("TypeNameLink");
    
    template <typename PoseT>
    PoseT const& Link<PoseT>::getTipToRootPose()  
    {
        tipToRootFrame_q0 = grs::compose(rootFrame_q0.inverse2(),tipFrame_q0);
        return tipToRootFrame_q0; 
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
    
    template <typename PoseT>
    class AttachmentFrame
    {
        public:
            AttachmentFrame()=default;
            AttachmentFrame(PoseT const& pose, FrameType const& type):poseData(pose), tagData(type)
            {
            };
            PoseT poseData;
            FrameType tagData;
    };
    
    template <typename PoseT>
    AttachmentFrame<PoseT> createAttachmentFrame(PoseT const& pose, FrameType const& type)
    {
        
        return AttachmentFrame<PoseT>(pose, type);
    };
    
    template <typename PoseT>
    class Segment
    {
        public:
            Segment(const std::string &givenName, typename ParameterTypeQualifier< Link<PoseT> >::RefToConstT linkOfSegment, std::vector< AttachmentFrame<PoseT> > const& listOfFrames)
            {
                classData.instanceName = givenName;
                classData.instanceID = uuid_generator(classData.instanceName);
                attachmentFrames = listOfFrames;
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
            
            
            //return the name, uuid, and classuuid
            std::string const& getName() const {return classData.instanceName;};
            boost::uuids::uuid const& getUID() const {return classData.instanceID;};
            boost::uuids::uuid const& getClassUID() const {return modelID;};
            
            ~Segment(){};
        private:
            std::vector< AttachmentFrame<PoseT> > attachmentFrames;
            struct MetaData classData;
            static boost::uuids::uuid modelID;
    };
    
    template <typename PoseT >
    boost::uuids::uuid Segment<PoseT>::modelID = uuid_generator("TypeNameSegment");
    
    /**
     * \brief Joint specification requires two segment instances and pose information of joint attachment frames on each of these segments
     *
     */
    template <typename PoseT>
    class Joint
    {
        public:
            
            Joint(const std::string &givenName, AttachmentFrame<PoseT> const& firstSegmentJointFrame, AttachmentFrame<PoseT> const& secondSegmentJointFrame, const JointProperties &properties )
            {
                classData.instanceName = givenName;
                classData.instanceID = uuid_generator(classData.instanceName);
                predecessorFrame = firstSegmentJointFrame;
                successorFrame = secondSegmentJointFrame;
                jointprops = properties;
            };
            
            /**
        	 * \brief compute and return relative pose of the tip of the given segment with respect to its predecessor's tip frame
        	 *
        	 * \param tipFramePose
        	 */
            void getSegmentTipPose(typename ParameterTypeQualifier<PoseT>::RefToArgT tipFramePose);
            
            /**
        	 * \brief compute and return relative pose between to joint frames of two segments
        	 *
        	 * \param jointFramePose is a vector of doubles for multiDoF, for 1 DoF it is of size 1
        	 */
            
            bool getPose(std::vector<double> const& jointvalues, typename ParameterTypeQualifier<PoseT>::RefToArgT currentPose);
            
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
            
            //return the name, uuid, and classuuid
            std::string const& getName() const {return classData.instanceName;};
            boost::uuids::uuid const& getUID() const {return classData.instanceID;};
            boost::uuids::uuid const& getClassUID() const {return modelID;};
            ~Joint(){};
        private:
            struct MetaData classData;
            static boost::uuids::uuid modelID;
            AttachmentFrame<PoseT> predecessorFrame;
            AttachmentFrame<PoseT> successorFrame;
            JointProperties jointprops;
        protected:
            bool getPoseImpl(std::vector<double> const& jointvalues, typename ParameterTypeQualifier<PoseT>::RefToArgT posetochange);
            bool getSegmentTipPoseImpl(typename ParameterTypeQualifier<PoseT>::RefToArgT tipFramePose);
    };
    
    template <typename PoseT>
    boost::uuids::uuid Joint<PoseT>::modelID = uuid_generator("TypeNameJoint");
    
    template <typename PoseT>
    void Joint<PoseT>::getSegmentTipPose(typename ParameterTypeQualifier<PoseT>::RefToArgT tipFramePose)
    {
        if(!getSegmentTipPoseImpl(tipFramePose))
            std::cout << "Inside Joint Segment Pose Impl " << tipFramePose <<std::endl;
        return;
    }
    
    template <typename PoseT>
    bool Joint<PoseT>::getPose(std::vector<double> const& jointvalues, typename ParameterTypeQualifier<PoseT>::RefToArgT currentPose)
    {
        if (getPoseImpl(jointvalues, currentPose))
            std::cout << "Inside Joint Pose Impl " << currentPose <<std::endl;
        return true;
    }
    
    template <>
    bool Joint< grs::Pose<KDL::Vector, KDL::Rotation> >::getPoseImpl(std::vector<double> const& jointvalues, grs::Pose<KDL::Vector, KDL::Rotation> &posetochange)
    {
        posetochange = grs::compose(predecessorFrame.poseData.inverse2(),successorFrame.poseData);
        grs::OrientationCoordinates<KDL::Rotation> newOrientationCoord = KDL::Rotation::RotZ(jointvalues[0]);
        return true;
    }
    
    template<>
    bool Joint< grs::Pose<KDL::Vector, KDL::Rotation> >::getSegmentTipPoseImpl(grs::Pose<KDL::Vector, KDL::Rotation> &tipFramePose)
    {
        
        return true;
    }
    /**
     * \brief  Chain specification requires a sequence of  joint instances
     *
     */
    template <typename PoseT, typename ContainerT = std::vector< Joint<PoseT> > >
    class Chain
    {
        public:
            Chain(std::string const& givenName, ContainerT const& jointlist)
            {
                classData.instanceName = givenName;
                classData.instanceID = uuid_generator(classData.instanceName);
            };
            /**
             * \brief  return true if the constructed chain is valid
             *
             */
            bool isChainValid();
            
            //return the name, uuid, and classuuid
            std::string const& getName() const {return classData.instanceName;};
            boost::uuids::uuid const& getUID() const {return classData.instanceID;};
            boost::uuids::uuid const& getClassUID() const {return modelID;};
            ~Chain(){};
        private:
            struct MetaData classData;
            static boost::uuids::uuid modelID;
    
    };
    
    template <typename PoseT, typename ContainerT>
    boost::uuids::uuid Chain<PoseT, ContainerT >::modelID = uuid_generator("TypeNameChain");
    
    template <typename PoseT, typename ContainerT>
    bool Chain<PoseT, ContainerT >::isChainValid()
    { 
        return true;
    };
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
