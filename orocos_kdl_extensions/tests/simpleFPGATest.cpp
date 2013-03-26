
#define VERBOSE_CHECK //switches on console output in kdl related methods
// #define VERBOSE_CHECK_MAIN // switches on console output in main

//#include <kdl_extensions/functionalcomputation_kdltypes.hpp>
#include <kdl_extensions/functionalcomputation_util.hpp>
#include <iostream>

using namespace std;
using namespace kdle;



template <typename OperationT, int N>
class OperationTDerived : public OperationT
{
public:

    OperationTDerived(typename ParameterTypeQualifier<OperationT>::RefToT a_op) : OperationT(a_op)
    {
    };

    OperationTDerived(typename ParameterTypeQualifier<OperationT>::RefToConstT a_op) : OperationT(a_op)
    {
    };
};

//composition template
//later should enable any function with any number of parameters

template <typename OperationT1, typename OperationT2> //could be transform, project category of operations
class Composite : private OperationTDerived<OperationT1, 1 >, private OperationTDerived<OperationT2, 2 >
{
public:

    enum
    {
        NumberOfParams = OperationT2::NumberOfParams
    };

    typedef typename OperationT1::ReturnType ReturnType;

    typedef typename OperationTParameterType<OperationT2, 1 > ::Type Param1T;
    typedef typename OperationTParameterType<OperationT2, 2 > ::Type Param2T;
    typedef typename OperationTParameterType<OperationT2, 3 > ::Type Param3T;
    typedef typename OperationTParameterType<OperationT2, 4 > ::Type Param4T;
    typedef typename OperationTParameterType<OperationT2, 5 > ::Type Param5T;
    typedef typename OperationTParameterType<OperationT2, 6 > ::Type Param6T;
    typedef typename OperationTParameterType<OperationT2, 7 > ::Type Param7T;

    Composite(typename ParameterTypeQualifier<OperationT1>::RefToConstT a_p1,
              typename ParameterTypeQualifier<OperationT2>::RefToConstT a_p2) :
              OperationTDerived<OperationT1, 1 > (a_p1), OperationTDerived<OperationT2, 2 > (a_p2)
    {
    };

    //overloaded for one param
    //compose(f,g)(t) will give the following result
    //f(x), where x = g(t)
    inline ReturnType operator()(typename ParameterTypeQualifier<Param1T>::RefToConstT a_param1)
    {
        return OperationTDerived<OperationT1, 1 > ::operator()(OperationTDerived<OperationT2, 2 > ::operator()(a_param1));
    };
    //overloaded for two params
    //compose(f,g)(x,y) will give the following result
    //f(x,z), where z = g(x,y)
    inline ReturnType operator()(typename ParameterTypeQualifier<Param1T>::RefToConstT a_param1,
                                 typename ParameterTypeQualifier<Param2T>::RefToConstT a_param2)
    {
        std::cout << "2 argument composition call" << std::endl;
        return OperationTDerived<OperationT1, 1 > ::operator()(a_param1, OperationTDerived<OperationT2, 2 > ::operator()(a_param1, a_param2));
    };

    //overloaded for three params
    //compose(f,g)(x,y,t) will give the following result
    //f(x,y,z) where z = g(x,y,t)
    inline ReturnType operator()(typename ParameterTypeQualifier<Param1T>::RefToConstT a_param1,
                                 typename ParameterTypeQualifier<Param2T>::RefToConstT a_param2,
                                 typename ParameterTypeQualifier<Param3T>::RefToConstT a_param3)
    {
        std::cout << "3 argument composition call" << std::endl;
        return OperationTDerived<OperationT1, 1 > ::operator()(a_param1, a_param2, OperationTDerived<OperationT2, 2 > ::operator()(a_param1, a_param2, a_param3));
    };

     //overloaded for four params
    //compose(f,g)(x,y,z,t) will give the following result
    //f(x,y,z,w) where w = g(x,y,z,t)
    inline ReturnType operator()(typename ParameterTypeQualifier<Param1T>::RefToConstT a_param1,
                                 typename ParameterTypeQualifier<Param2T>::RefToConstT a_param2,
                                 typename ParameterTypeQualifier<Param3T>::RefToConstT a_param3,
                                 typename ParameterTypeQualifier<Param4T>::RefToConstT a_param4)
    {
        std::cout << "4 argument composition call" << std::endl;
        return OperationTDerived<OperationT1, 1 > ::operator()(a_param1, a_param2, a_param3, OperationTDerived<OperationT2, 2 > ::operator()(a_param1, a_param2, a_param3, a_param4));
    };

    inline ReturnType operator()(typename ParameterTypeQualifier<Param1T>::RefToConstT a_param1,
                                 typename ParameterTypeQualifier<Param2T>::RefToConstT a_param2,
                                 typename ParameterTypeQualifier<Param3T>::RefToConstT a_param3,
                                 typename ParameterTypeQualifier<Param4T>::RefToArgT a_param4,
                                 typename ParameterTypeQualifier<Param5T>::RefToConstT a_param5)
    {
        std::cout << "5 argument composition call" << std::endl;
        return OperationTDerived<OperationT1, 1 > ::operator()(a_param1, a_param2, a_param3, a_param4, OperationTDerived<OperationT2, 2 > ::operator()(a_param1, a_param2, a_param3, a_param4, a_param5));
    };

    //can add further overloads when needed


};

//convinience function for composition functor

template <typename OperationT1, typename OperationT2>
inline Composite<OperationT1, OperationT2> compose(OperationT1 a_p1, OperationT2 a_p2)
{
    return Composite<OperationT1, OperationT2 > (a_p1, a_p2);
};




template<typename T1, typename T2>
class simplefpgafunction
{
public:
    enum
    {
        NumberOfParams = 2
    };
    typedef T1 ReturnType;
    typedef T1 Param1T;
    typedef T2 Param2T;

    inline ReturnType operator()(typename ParameterTypeQualifier<Param1T>::RefToConstT param1,
                                 typename ParameterTypeQualifier<Param2T>::RefToConstT param2)
    {
        internalValue = param1+param2;
        std::cout << internalValue << std::endl;
        return internalValue;
    };
private:
    ReturnType internalValue;
};




int main(int argc, char** argv)
{
    simplefpgafunction<int, int> func1;
    simplefpgafunction<long, long> func2;

    compose(func2, func1)(100,10);

    return 0;
}




