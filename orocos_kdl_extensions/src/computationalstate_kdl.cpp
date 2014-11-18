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


#include <kdl_extensions/computationalstate_kdl.hpp>
namespace kdle

{
//Link computational state

//TODO: These has to be deprecated in the favor of Global and Local
//Link computational state

SegmentState::SegmentState()
{
    X.Identity();
    Xtotal.Identity();
    Xdot.Zero();
    Xdotdot.Zero();
    F.Zero();
//    Fext.Zero();
    Z.Zero();
    Vj.Zero();
    jointIndex = 0;
    jointName = " ";
    segmentName = " ";

}

SegmentState::SegmentState(const SegmentState& copy)
{
    X = copy.X;
    Xtotal = copy.Xtotal;
    Xdot = copy.Xdot;
    Xdotdot = copy.Xdotdot;
    F = copy.F;
//    Fext = copy.Fext;
    Z = copy.Z;
    Vj = copy.Vj;
    jointIndex = copy.jointIndex;
    jointName = copy.jointName;
    segmentName = copy.segmentName;
}

SegmentState& SegmentState::operator=(const SegmentState& copy)
{
    if (this != &copy)
    {
        X = copy.X;
        Xtotal = copy.Xtotal;
        Xdot = copy.Xdot;
        Xdotdot = copy.Xdotdot;
        F = copy.F;
//        Fext = copy.Fext;
        Z = copy.Z;
        Vj = copy.Vj;
        jointIndex = copy.jointIndex;
        jointName = copy.jointName;
        segmentName = copy.segmentName;
    }
    return *this;
}

bool SegmentState::operator==(const SegmentState& instance)
{
    return
    ((this->X == instance.X) &&
            (this->Xdot == instance.Xdot) &&
            (this->Xdotdot == instance.Xdotdot) &&
            (this->F == instance.F));
};

bool SegmentState::operator!=(const SegmentState& instance)
{
    return !(operator==(instance));
};

SegmentState::~SegmentState()
{


}


//Joint computational state

JointState::JointState()
{
    q = 0.0;
    qdot = 0.0;
    qdotdot = 0.0;
    torque = 0.0;
    jointIndex = 0;
    Fext.Zero();
    jointName = " ";

}

JointState::JointState(const JointState& copy)
{
    q = copy.q;
    qdot = copy.qdot;
    qdotdot = copy.qdotdot;
    torque = copy.torque;
    Fext = copy.Fext;
    jointIndex = copy.jointIndex;
    jointName = copy.jointName;
}

JointState& JointState::operator=(const JointState& copy)
{
    if (this != &copy)
    {
        q = copy.q;
        qdot = copy.qdot;
        qdotdot = copy.qdotdot;
        torque = copy.torque;
        Fext = copy.Fext;
        jointIndex = copy.jointIndex;
        jointName = copy.jointName;
    }
    return *this;
}

JointState::~JointState()
{


}

};