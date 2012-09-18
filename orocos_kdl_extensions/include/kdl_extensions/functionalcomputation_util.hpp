/* 
 * File:   functionalcomputation_util.hpp
 * Author: azamat
 *
 * Created on September 18, 2012, 3:59 PM
 */

#ifndef FUNCTIONALCOMPUTATION_UTIL_HPP
#define	FUNCTIONALCOMPUTATION_UTIL_HPP

// primary template: yield second or third argument depending on first argument
template<bool C, typename Ta, typename Tb>
class IfThenElse;

// partial specialization: true yields second argument
template<typename Ta, typename Tb>
class IfThenElse<true, Ta, Tb> {
  public:
    typedef Ta ResultT;
};

// partial specialization: false yields third argument
template<typename Ta, typename Tb>
class IfThenElse<false, Ta, Tb> {
  public:
    typedef Tb ResultT;
};


template <typename T>
class TypeOp {            // primary template
  public:
    typedef T         ArgT;
    typedef T         BareT;
    typedef T const   ConstT;
    typedef T &       RefT;
    typedef T &       RefBareT;
    typedef T const & RefConstT;
};
/*
template<typename T>
class ForwardParamT {
  public:
    typedef typename IfThenElse<TypeT<T>::IsClassT, // this should be changed to adjust for our use case
                                typename TypeOp<T>::RefConstT,
                                typename TypeOp<T>::ArgT
                               >::ResultT
            Type;
};

template<>
class ForwardParamT<void> {
  private:
    class Unused {};
  public:
    typedef Unused Type;
};
*/
#endif	/* FUNCTIONALCOMPUTATION_UTIL_HPP */

