#pragma once
#include "matrix.h"
#include <type_traits>

namespace Engine
{
    template <typename T, typename Enable>
    struct DynamicMatrix;

    template <typename Type>
    struct DynamicMatrix<Type, typename std::enable_if<std::is_arithmetic<Type>::value>::type>;

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
            return data[this->cols * target_row + target_col].at(row, col);
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

        DynamicMatrix<typename find_type<T>::type> dense()
        {
            Point2i size = this->get_size();
            using Type = typename find_type<T>::type;
            Type *new_data = (Type *)malloc(rows * cols * sizeof(Type));
            for (int i = 0; i < rows; i++)
                for (int j = 0; j < cols; j++)
                    new_data[i * cols + j] = this->at(i, j);

            DynamicMatrix<Type> dense_matrix(size[0], size[1]);
            free(dense_matrix.data);
            dense_matrix.data = new_data;
            return std::move(dense_matrix);
        }
    };

    template <typename Type>
    struct DynamicMatrix<Type, typename std::enable_if<std::is_arithmetic<Type>::value>::type>
    {
        int rows;
        int cols;
        Type *data;
        using value_type = Type;
        DynamicMatrix(int r, int c) : rows(r), cols(c)
        {
            data = (Type *)malloc(rows * cols * sizeof(Type));
            memset(data, 0, rows * cols * sizeof(Type));
        }

        template <int rows, int cols>
        DynamicMatrix(Matrix<Type, rows, cols> x) : rows(rows), cols(cols)
        {
            data = (Type *)malloc(rows * cols * sizeof(Type));
            for (int i = 0; i < rows * cols; i++)
                data[i] = x[i];
        }

        DynamicMatrix(DynamicMatrix<Type> &&m)
        {
            this->rows = m.rows;
            this->cols = m.cols;
            this->data = m.data;
            m.data = nullptr;
        }

        void _impl_args(int index, const Type &t)
        {
            this->data[index] = t;
        }

        template <typename... Args>
        void _impl_args(int index, const Type &t, const Args &...rest)
        {
            this->data[index] = t;
            _impl_args(index + 1, rest...);
        }

        template <typename... Args>
        DynamicMatrix(const Type &t, const Args &...rest) : rows(1), cols(sizeof...(rest) + 1)
        {

            this->data = (Type *)malloc((sizeof...(rest) + 1) * sizeof(Type));
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

        DynamicMatrix<Type> set_size(int r, int c)
        {
            this->rows = r;
            this->cols = c;
            return std::move(*this);
        }

        DynamicMatrix<Type> T()
        {
            DynamicMatrix<Type> m(cols, rows);
            for (int r = 0; r < cols; r++)
                for (int c = 0; c < rows; c++)
                    m.data[r * rows + c] = this->data[c * cols + r];

            return std::move(m);
        }

        Type at(int row, int col)
        {
            return data[row * cols + col];
        }

        void custom_free()
        {
            free(data);
        }

        ~DynamicMatrix()
        {
            this->custom_free();
        }

        DynamicMatrix<Type>  operator*(DynamicMatrix<Type> &b)
        {
            assert(this->cols == b.rows);
            auto res = DynamicMatrix<Type>(rows,b.cols);
            for (int m = 0; m < rows; m++)
                for (int s = 0; s < b.cols; s++)
                {
                    res.data[m * b.cols + s] = 0;
                    for (int n = 0; n < b.rows; n++)
                        res.data[m * b.cols + s] += this->data[m * cols + n] * b.data[n * b.cols + s];
                }
            return res;
        }
    };

    inline void solve(DynamicMatrix<double> A, DynamicMatrix<double> b, DynamicMatrix<double> &x)
    {
        int n = A.rows;
        if (A.cols != n || b.cols != 1 || b.rows != n)
        {
            throw std::invalid_argument("Incompatible dimensions for Cholesky solve.");
        }

        DynamicMatrix<double> L(n, n);
        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j <= i; ++j)
            {
                double sum = 0.0;
                for (int k = 0; k < j; ++k)
                    sum += L.at(i, k) * L.at(j, k);

                if (i == j)
                {
                    double diag = A.at(i, i) - sum;
                    if (diag <= 0.0)
                        throw std::runtime_error("Matrix is not positive definite.");
                    L.data[i * n + j] = std::sqrt(diag);
                }
                else
                {
                    L.data[i * n + j] = (A.at(i, j) - sum) / L.at(j, j);
                }
            }
        }

        DynamicMatrix<double> y(n, 1);
        for (int i = 0; i < n; ++i)
        {
            double sum = 0.0;
            for (int k = 0; k < i; ++k)
                sum += L.at(i, k) * y.at(k, 0);
            y.data[i] = (b.at(i, 0) - sum) / L.at(i, i);
        }

        for (int i = n - 1; i >= 0; --i)
        {
            double sum = 0.0;
            for (int k = i + 1; k < n; ++k)
                sum += L.at(k, i) * x.at(k, 0);
            x.data[i] = (y.at(i, 0) - sum) / L.at(i, i);
        }
    }

}