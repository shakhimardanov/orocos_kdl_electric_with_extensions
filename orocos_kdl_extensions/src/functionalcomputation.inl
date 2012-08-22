#ifndef FUNCTIONALCOMPUTATION_HPP
#define	FUNCTIONALCOMPUTATION_HPP




<typename T_Operation1, typename T_Operation2, typename T_IterationElement = KDL::SegmentMap::const_iterator,
          typename T_ComputationalState1 = KDL::JointState, typename T_ComputationalState2 = KDL::SegmentState>
iterateOver_t<T_Operation1, T_Operation2, T_IterationElement, T_ComputationalState1, T_ComputationalState2>::
iterateOver_t()
{

}


<typename T_Operation1, typename T_Operation2, typename T_IterationElement = KDL::SegmentMap::const_iterator,
          typename T_ComputationalState1 = KDL::JointState, typename T_ComputationalState2 = KDL::SegmentState>
T_ComputationalState2 iterateOver_t<T_Operation1, T_Operation2, T_IterationElement, T_ComputationalState1, T_ComputationalState2>::
                      operator()(T_IterationElement p1,T_ComputationalState1 p2,T_ComputationalState2 p3,T_Operation1 op1, T_Operation2 op2)
{
    a_internalState = op1(p1,p2,p3);
    a_internalState = op2(p1,p2,a_internalState);
    return a_internalState;

}

<typename T_Operation1, typename T_Operation2, typename T_IterationElement = KDL::SegmentMap::const_iterator,
          typename T_ComputationalState1 = KDL::JointState, typename T_ComputationalState2 = KDL::SegmentState>
T_ComputationalState2 iterateOver_t<T_Operation1, T_Operation2, T_IterationElement, T_ComputationalState1, T_ComputationalState2>::
                      operator()(T_IterationElement p1,T_ComputationalState1 p2,T_ComputationalState2 p3,T_Operation1 op1)
{

    a_internalState = op1(p1, p2, p3);
    return a_internalState;

}





//template <typename T1, typename T2, typename T3, typename OP1, typename OP2>
//inline interateOver_t<T1, T2, T3, OP1, OP2> iterateOver()




#endif	/* FUNCTIONALCOMPUTATION_HPP */
