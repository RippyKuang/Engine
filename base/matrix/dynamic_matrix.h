#pragma once
#include "matrix.h"
#include <type_traits>

namespace Engine
{

    template <typename inst, template <typename...> typename tmpl>
    struct is_instantiation_of : std::false_type {};
    
    template <template <typename...> typename impl, typename... args>
    struct is_instantiation_of<impl<args...>, impl> : std::true_type {};

    template <typename T>
    struct DynamicMatrix
    {
        int rows;
        int cols;
        T *data;
        DynamicMatrix(int r, int c) : rows(r), cols(c)
        {
            data = (T *)malloc(rows * cols * sizeof(T));
        } 
    };

    template <typename T,typename enable = void>
    struct find_type 
    {
        using type = T;
    };
    
    template <typename T>
    struct find_type<T, typename std::enable_if<is_instantiation_of<T, DynamicMatrix>::value>::type>
    {
        using type = typename find_type<T>::type;
    };

    
}