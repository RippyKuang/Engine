#pragma once
#include "matrix.h"
#include <type_traits>

namespace Engine
{

    template <typename inst, template <typename...> typename tmpl>
    struct is_instantiation_of : std::false_type
    {
    };

    template <template <typename...> typename impl, typename... args>
    struct is_instantiation_of<impl<args...>, impl> : std::true_type
    {
    };

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

        Point2i get_size()
        {
            int r = 0, c = 0;
            for (int i = 0; i < cols; i++)
                c += this->data[i].get_size()[1];
            for (int i = 0; i < rows; i++)
                r += this->data[i].get_size()[0];
            return Point2i(r, c);
        }

        T* at(int row, int col)
        {
            return &data[row * cols + col];
        }
    };

    template <>
    struct DynamicMatrix<double>
    {
        int rows;
        int cols;
        double *data;
        DynamicMatrix(int r, int c) : rows(r), cols(c)
        {
            data = (double *)malloc(rows * cols * sizeof(double));
        }

        Point2i get_size()
        {
            return Point2i(rows, cols);
        }

    };

    template <typename T, typename enable = void>
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