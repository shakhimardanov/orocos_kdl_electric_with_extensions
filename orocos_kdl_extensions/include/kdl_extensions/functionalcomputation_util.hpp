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
 * //this is related to Parameter template defined in functionalcomputation.hpp
template <typename F, int N>
class UsedFunctorParam; // it is the same as Parameter template. It identifies type of the function argument

template <typename F, int N>
class FunctorParam {
  private:
    class Unused {
      private:
        class Private {};
      public:
        typedef Private Type;
    };
  public:
    typedef typename IfThenElse<F::NumParams>=N,
                                UsedFunctorParam<F,N>,
                                Unused>::ResultT::Type
            Type;
};
 *
 *
 * // this template makes passed function arguments read-only (as references to const value)
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

 *
 *

//this template is used to wrap normal function pointer into function objects
 // primary template handles maximum number of parameters:
template<typename RT, typename P1 = void,
                      typename P2 = void,
                      typename P3 = void>
class FunctionPtrT {
  public:
    enum { NumParams = 3 };
    typedef RT (*Type)(P1,P2,P3);
};

// partial specialization for two parameters:
template<typename RT, typename P1,
                      typename P2>
class FunctionPtrT<RT, P1, P2, void> {
  public:
    enum { NumParams = 2 };
    typedef RT (*Type)(P1,P2);
};

// partial specialization for one parameter:
template<typename RT, typename P1>
class FunctionPtrT<RT, P1, void, void> {
  public:
    enum { NumParams = 1 };
    typedef RT (*Type)(P1);
};

// partial specialization for no parameters:
template<typename RT>
class FunctionPtrT<RT, void, void, void> {
  public:
    enum { NumParams = 0 };
    typedef RT (*Type)();
};
 *
 *

 * //this wrap the function into a class functor and adds additional information
 * // on argument, function call return types and number of arguments
 * template<typename RT, typename P1 = void,
                      typename P2 = void,
                      typename P3 = void>
class FunctionPtr {
  private:
    typedef typename FunctionPtrT<RT,P1,P2,P3>::Type FuncPtr;
    // the encapsulated pointer:
    FuncPtr fptr;
  public:
    // to fit in our framework:
    enum { NumParams = FunctionPtrT<RT,P1,P2,P3>::NumParams };
    typedef RT ReturnT;
    typedef P1 Param1T;
    typedef P2 Param2T;
    typedef P3 Param3T;

    // constructor:
    FunctionPtr(FuncPtr ptr)
     : fptr(ptr) {
    }

    // ``function calls'':
    RT operator()() {
        return fptr();
    }
    RT operator()(typename ForwardParamT<P1>::Type a1) {
        return fptr(a1);
    }
    RT operator()(typename ForwardParamT<P1>::Type a1,
                  typename ForwardParamT<P2>::Type a2) {
        return fptr(a1, a2);
    }
    RT operator()(typename ForwardParamT<P1>::Type a1,
                  typename ForwardParamT<P2>::Type a2,
                  typename ForwardParamT<P3>::Type a3) {
        return fptr(a1, a2, a3);
    }
};
 *
 *
 *
 *
//these are convinience fucntions for the template above
// to make argument deduction work.
 * template<typename RT> inline
FunctionPtr<RT> func_ptr (RT (*fp)())
{
    return FunctionPtr<RT>(fp);
}

template<typename RT, typename P1> inline
FunctionPtr<RT,P1> func_ptr (RT (*fp)(P1))
{
    return FunctionPtr<RT,P1>(fp);
}

template<typename RT, typename P1, typename P2> inline
FunctionPtr<RT,P1,P2> func_ptr (RT (*fp)(P1,P2))
{
    return FunctionPtr<RT,P1,P2>(fp);
}

template<typename RT, typename P1, typename P2, typename P3> inline
FunctionPtr<RT,P1,P2,P3> func_ptr (RT (*fp)(P1,P2,P3))
{
    return FunctionPtr<RT,P1,P2,P3>(fp);
}

*/
#endif	/* FUNCTIONALCOMPUTATION_UTIL_HPP */

