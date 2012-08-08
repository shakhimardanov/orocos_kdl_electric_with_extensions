#ifndef FUNCTIONALCOMPUTATION_HPP
#define	FUNCTIONALCOMPUTATION_HPP

template <typename T1, typename T2, typename T3, typename OP1, typename OP2>
iterateOver_t<T1, T2, T3, OP1, OP2>::iterateOver_t()
{

}

template <typename T1, typename T2, typename T3, typename OP1, typename OP2>
T3 iterateOver_t<T1, T2, T3, OP1, OP2>::operator() (T1 p1,T2 p2,T3 p3,OP2 op2, OP1 op1)
{
    a_internalState = op1(p1,p2,p3);
    a_internalState = op2(p1,p2,a_internalState);
    return a_internalState;
}

template <typename T1, typename T2, typename T3, typename OP1, typename OP2>
T3 iterateOver_t<T1, T2, T3, OP1, OP2>::operator() (T1 p1,T2 p2,T3 p3,OP1 op1)
{
    a_internalState = op1(p1, p2, p3);
    return a_internalState;
}

//template <typename T1, typename T2, typename T3, typename OP1, typename OP2>
//inline interateOver_t<T1, T2, T3, OP1, OP2> iterateOver()

template <typename T2, typename T3, typename OP1, typename OP2>
iterateOver_t<KDL::SegmentMap::const_iterator, T2, T3, OP1, OP2>::iterateOver_t()
{

}

T3 iterateOver_t<KDL::SegmentMap::const_iterator, T2, T3, OP1, OP2>::operator()(KDL::SegmentMap::const_iterator,T2,T3,OP1, OP2)
{


}

T3 iterateOver_t<KDL::SegmentMap::const_iterator, T2, T3, OP1, OP2>::operator()(KDL::SegmentMap::const_iterator,T2,T3,OP1)
{

}



#endif	/* FUNCTIONALCOMPUTATION_HPP */
