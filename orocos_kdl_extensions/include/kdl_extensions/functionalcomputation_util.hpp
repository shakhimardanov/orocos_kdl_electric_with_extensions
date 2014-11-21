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

#ifndef FUNCTIONALCOMPUTATION_UTIL_HPP
#define	FUNCTIONALCOMPUTATION_UTIL_HPP


namespace kdle
{
// primary template: yield second or third argument depending on first argument
template<bool Choice, typename FirstType, typename SecondType>
class Selector;

// partial specialization: true yields second argument and false yields the third arg

template<typename FirstType, typename SecondType>
class Selector < true, FirstType, SecondType>
{
public:
    typedef FirstType ResultType;
};

template<typename FirstType, typename SecondType>
class Selector < false, FirstType, SecondType>
{
public:
    typedef SecondType ResultType;
};

template <typename OperationT, int N>
class Parameter;

//partial specialization
#define ParameterT(N)                                           \
        template<typename OperationT>                           \
        class Parameter<OperationT,N>{                          \
            public:                                             \
                typedef typename OperationT::Param##N##T Type;  \
        }

ParameterT(1);
ParameterT(2);
ParameterT(3);
ParameterT(4);
ParameterT(5);
ParameterT(6);
ParameterT(7);
#undef ParameterT

template <typename OperationT, int N>
class OperationTParameterType
{
private:

    class NotUsedParameter
    {
    private:

        class DymmyType
        {
        };
    public:
        typedef DymmyType Type;
    };
public:
    typedef typename Selector<OperationT::NumberOfParams >= N, Parameter<OperationT, N>, NotUsedParameter>::ResultType::Type Type;
};


// primary template for argument qualifiers

template <typename T>
class ParameterTypeQualifier
{
public:
    typedef T ArgT;
    typedef T const ConstT;
    typedef T & RefToT;
    typedef T & RefToArgT;
    typedef T const & RefToConstT;
};


//this template is used to parameterize function pointer types. This is then used
// for function pointer wrapper which actually creates a class type functor.
// primary template handles maximum number of parameters. Here it is seven parameters.
//if parameters type is void it means that it is not used/empty

template< typename ReturnType,
        typename Param1 = void,
        typename Param2 = void,
        typename Param3 = void,
        typename Param4 = void,
        typename Param5 = void,
        typename Param6 = void,
        typename Param7 = void>
class FunctionPointerT
{
public:

    enum
    {
        NumberofParams = 7
    };
    typedef ReturnType(*Type)(Param1, Param2, Param3, Param4, Param5, Param6, Param7);
};


// partial specialization for six parameters:
template<typename ReturnType, typename Param1, typename Param2, typename Param3,
                              typename Param4, typename Param5, typename Param6 >
class FunctionPointerT<ReturnType, Param1, Param2, Param3, Param4, Param5, Param6, void>
{
  public:
    enum { NumberOfParams = 6 };
    typedef ReturnType(*Type)(Param1, Param2, Param3, Param4, Param5, Param6);
};

// partial specialization for five parameter:
template<typename ReturnType, typename Param1, typename Param2, typename Param3,
                              typename Param4, typename Param5>
class FunctionPointerT<ReturnType, Param1, Param2, Param3, Param4, Param5, void, void>
{
  public:
    enum { NumberOfParams = 5 };
    typedef ReturnType(*Type)(Param1, Param2, Param3, Param4, Param5);
};

// partial specialization for four parameters:
template<typename ReturnType, typename Param1, typename Param2, typename Param3, typename Param4 >
class FunctionPointerT<ReturnType, Param1, Param2, Param3, Param4, void, void, void>
{
  public:
    enum { NumberOfParams = 4 };
    typedef ReturnType(*Type)(Param1, Param2, Param3, Param4);
};


// partial specialization for three parameters:
template<typename ReturnType, typename Param1, typename Param2, typename Param3>
class FunctionPointerT<ReturnType, Param1, Param2, Param3, void, void, void, void>
{
  public:
    enum { NumberOfParams = 3 };
    typedef ReturnType(*Type)(Param1, Param2, Param3);
};


// partial specialization for two parameters:
template<typename ReturnType, typename Param1, typename Param2>
class FunctionPointerT<ReturnType, Param1, Param2, void, void, void, void, void>
{
  public:
    enum { NumberOfParams = 2 };
    typedef ReturnType(*Type)(Param1, Param2);
};


// partial specialization for one parameters:
template<typename ReturnType, typename Param1>
class FunctionPointerT<ReturnType, Param1, void, void, void, void, void, void>
{
  public:
    enum { NumberOfParams = 1 };
    typedef ReturnType(*Type)(Param1);
};


// partial specialization for no parameters:
template<typename ReturnType>
class FunctionPointerT<ReturnType, void, void, void, void, void, void, void>
{
  public:
    enum { NumberOfParams = 0 };
    typedef ReturnType(*Type)();
};

//this wraps the function pointer into a class functor and adds additional information
// on argument, function call return types and number of arguments
template< typename ReturnT,
        typename Param1 = void,
        typename Param2 = void,
        typename Param3 = void,
        typename Param4 = void,
        typename Param5 = void,
        typename Param6 = void,
        typename Param7 = void>

class FunctionPointerToFunctor
{
  private:
    typedef typename FunctionPointerT<ReturnT,Param1,Param2,Param3,Param4,Param5,Param6,Param7>::Type FuncPtr;
    // the encapsulated pointer:
    FuncPtr funcptr;
  public:
    // to fit in our framework:
    enum { NumParams = FunctionPointerT<ReturnT,Param1,Param2,Param3,Param4,Param5,Param6,Param7>::NumberOfParams };
    //these are used by composition template to identify return types are of the operation/composite operation
    //and paramter types of operations(functor) being composed.
    typedef ReturnT ReturnType;
    typedef Param1 Param1T;
    typedef Param2 Param2T;
    typedef Param3 Param3T;
    typedef Param4 Param4T;
    typedef Param5 Param5T;
    typedef Param6 Param6T;
    typedef Param7 Param7T;

    // constructor:
    FunctionPointerToFunctor(FuncPtr ptr) : funcptr(ptr)
    {
    }

    // overloaded function call operators for different number of parameters
    ReturnT operator()()
    {
        return funcptr();
    }
    
    ReturnT operator()(typename ParameterTypeQualifier<Param1>::RefToConstT arg1)
    {
        return funcptr(arg1);
    }
    
    ReturnT operator()(typename ParameterTypeQualifier<Param1>::RefToConstT arg1,
                          typename ParameterTypeQualifier<Param2>::RefToConstT arg2)
    {
        return funcptr(arg1, arg2);
    }
    
    ReturnT operator()(typename ParameterTypeQualifier<Param1>::RefToConstT arg1,
                          typename ParameterTypeQualifier<Param2>::RefToConstT arg2,
                          typename ParameterTypeQualifier<Param3>::RefToConstT arg3)
    {
        return funcptr(arg1, arg2, arg3);
    }

    ReturnT operator()(typename ParameterTypeQualifier<Param1>::RefToConstT arg1,
                          typename ParameterTypeQualifier<Param2>::RefToConstT arg2,
                          typename ParameterTypeQualifier<Param3>::RefToConstT arg3,
                          typename ParameterTypeQualifier<Param4>::RefToConstT arg4)
    {
        return funcptr(arg1, arg2, arg3, arg4);
    }

    ReturnT operator()(typename ParameterTypeQualifier<Param1>::RefToConstT arg1,
                          typename ParameterTypeQualifier<Param2>::RefToConstT arg2,
                          typename ParameterTypeQualifier<Param3>::RefToConstT arg3,
                          typename ParameterTypeQualifier<Param4>::RefToConstT arg4,
                          typename ParameterTypeQualifier<Param5>::RefToConstT arg5)
    {
        return funcptr(arg1, arg2, arg3, arg4, arg5);
    }


    ReturnT operator()(typename ParameterTypeQualifier<Param1>::RefToConstT arg1,
                          typename ParameterTypeQualifier<Param2>::RefToConstT arg2,
                          typename ParameterTypeQualifier<Param3>::RefToConstT arg3,
                          typename ParameterTypeQualifier<Param4>::RefToConstT arg4,
                          typename ParameterTypeQualifier<Param5>::RefToConstT arg5,
                          typename ParameterTypeQualifier<Param6>::RefToConstT arg6)
    {
        return funcptr(arg1, arg2, arg3, arg4, arg5, arg6);
    }


    ReturnT operator()(typename ParameterTypeQualifier<Param1>::RefToConstT arg1,
                          typename ParameterTypeQualifier<Param2>::RefToConstT arg2,
                          typename ParameterTypeQualifier<Param3>::RefToConstT arg3,
                          typename ParameterTypeQualifier<Param4>::RefToConstT arg4,
                          typename ParameterTypeQualifier<Param5>::RefToConstT arg5,
                          typename ParameterTypeQualifier<Param6>::RefToConstT arg6,
                          typename ParameterTypeQualifier<Param7>::RefToConstT arg7)
    {
        return funcptr(arg1, arg2, arg3, arg4, arg5, arg6, arg7);
    }

};

//these are convinience fucntions for the template above
// to make argument deduction work.
 template<typename RT> inline
FunctionPointerToFunctor<RT> func_ptr (RT (*fp)())
{
    return FunctionPointerToFunctor<RT>(fp);
}

template<typename RT, typename P1> inline
FunctionPointerToFunctor<RT,P1> func_ptr (RT (*fp)(P1))
{
    return FunctionPointerToFunctor<RT,P1>(fp);
}

template<typename RT, typename P1, typename P2> inline
FunctionPointerToFunctor<RT,P1,P2> func_ptr (RT (*fp)(P1,P2))
{
    return FunctionPointerToFunctor<RT,P1,P2>(fp);
}

template<typename RT, typename P1, typename P2, typename P3> inline
FunctionPointerToFunctor<RT,P1,P2,P3> func_ptr (RT (*fp)(P1,P2,P3))
{
    return FunctionPointerToFunctor<RT,P1,P2,P3>(fp);
}



}
#endif	/* FUNCTIONALCOMPUTATION_UTIL_HPP */

