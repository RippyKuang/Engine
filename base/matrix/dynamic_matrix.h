#pragma once
#include "matrix.h"
#include <type_traits>

namespace Engine
{
    template <typename T, typename Enable>
    struct DynamicMatrix;

    template <typename inst, template <typename...> typename tmpl>
    struct is_instantiation_of : std::false_type
    {
    };

    template <template <typename...> typename impl, typename... args>
    struct is_instantiation_of<impl<args...>, impl> : std::true_type
    {
    };

    template <typename T, typename enable = void>
    struct find_type
    {
        using type = T;
    };

    template <typename T>
    struct find_type<T, typename std::enable_if<is_instantiation_of<T, DynamicMatrix>::value>::type>
    {
        using type = typename find_type<typename T::value_type>::type;
    };

    template <typename T, typename enable = void>
    struct DynamicMatrix
    {
        int rows;
        int cols;
        T *data;
        using value_type = T;
        DynamicMatrix(int r, int c) : rows(r), cols(c)
        {
            data = (T *)malloc(rows * cols * sizeof(T));
        }

        Point2i get_size()
        {
            int r = 0, c = 0;
            for (int i = 0; i < this->cols; i++)
                c += this->data[i].get_size()[1];
            for (int i = 0; i < rows; i++)
                r += this->data[i * this->cols].get_size()[0];
            return Point2i(r, c);
        }

        find_type<T>::type at(int row, int col)
        {
            int target_row = 0, target_col = 0;
            for (int i = 0; i < this->rows; i++)
            {
                if (row < data[i].get_size()[0])
                {
                    target_row = i;
                    break;
                }
                row -= data[i].get_size()[0];
            }
            for (int j = 0; j < this->cols; j++)
            {
                if (col < data[j].get_size()[1])
                {
                    target_col = j;
                    break;
                }
                col -= data[j].get_size()[1];
            }
            return data[this->rows * target_row + target_col].at(row, col);
        }

        void custom_free()
        {
            for (int i = 0; i < rows * cols; i++)
                data[i].custom_free();
            free(data);
        }

        ~DynamicMatrix()
        {
            this->custom_free();
        }
    };

    template <typename T>
    struct DynamicMatrix<T, typename std::enable_if<std::is_arithmetic<T>::value>::type>
    {
        int rows;
        int cols;
        T *data;
        using value_type = T;
        DynamicMatrix(int r, int c) : rows(r), cols(c)
        {
            data = (T *)malloc(rows * cols * sizeof(T));
        }

        template <int rows, int cols>
        DynamicMatrix(Matrix<T, rows, cols> x) : rows(rows), cols(cols)
        {
            data = (T *)malloc(rows * cols * sizeof(T));
            for (int i = 0; i < rows * cols; i++)
                data[i] = x[i];
        }

        DynamicMatrix(DynamicMatrix<T> &&m)
        {   
            this->rows = m.rows;
            this->cols = m.cols;
            this->data = m.data;
            m.data = nullptr;
        }

        void _impl_args(int index, const T &t)
        {
            this->data[index] = t;
        }

        template <typename... Args>
        void _impl_args(int index, const T &t, const Args &...rest)
        {
            this->data[index] = t;
            _impl_args(index + 1, rest...);
        }

        template <typename... Args>
        DynamicMatrix(const T &t, const Args &...rest): rows(1), cols(sizeof...(rest) + 1)
        {

            this->data = (T *)malloc((sizeof...(rest) + 1) * sizeof(T));
            this->data[0] = t;
            _impl_args(1, rest...);
        }

        template <typename _T, typename = typename std::enable_if<is_instantiation_of<_T, DynamicMatrix>::value>::type>
        DynamicMatrix &operator=(_T &&b)
        {

            this->data = b.data;
            this->rows = b.rows;
            this->cols = b.cols;
            b.data = nullptr;
            return *this;
        }

        Point2i get_size()
        {
            return Point2i(rows, cols);
        }
      
        DynamicMatrix set_size(int r, int c)
        {
            this->rows = r;
            this->cols = c;
            return std::move(*this);
        }

        T at(int row, int col)
        {
            return data[row * cols + col];
        }

        void custom_free()
        {
            free(data);
        }
    };

}