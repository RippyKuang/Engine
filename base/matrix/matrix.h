#pragma once
#include <tuple>
#include <iostream>
#include <assert.h>
#include <initializer_list>
#include <cstring>
#include <vector>
#include <immintrin.h>

#define EYE(x) Engine::eye<double, x>()
#define AXIS_X Vector3d{1, 0, 0}
#define AXIS_Y Vector3d{0, 1, 0}
#define AXIS_Z Vector3d{0, 0, 1}

namespace Engine
{
    template <typename _Scalar, int _Rows, int _Cols>
    class Matrix
    {
    protected:
        const int row = _Rows;
        const int col = _Cols;
        _Scalar *data;

    public:
        friend inline Matrix<double, 6, 1> operator+(Matrix<double, 6, 1> &a, Matrix<double, 6, 1> &&b);
        friend inline Matrix<double, 6, 1> operator+(Matrix<double, 6, 1> &a, Matrix<double, 6, 1> &b);
        Matrix(_Scalar *p) : data(p)
        {
        }
        Matrix(_Scalar val = 0)
        {
            data = (_Scalar *)malloc((_Rows * _Cols + _Rows * _Cols % 4) * sizeof(_Scalar));
            for (int i = 0; i < _Rows * _Cols; i++)
                data[i] = val;
        }

        Matrix(const Matrix<_Scalar, _Rows, _Cols> &m)
        {
            data = (_Scalar *)malloc((_Rows * _Cols + _Rows * _Cols % 4)* sizeof(_Scalar));
            for (int i = 0; i < _Rows * _Cols; i++)
                data[i] = m[i];
        }

        Matrix(Matrix<_Scalar, _Rows, _Cols> &&m)
        {

            this->data = m.data;
            m.data = nullptr;
        }
        template <typename _mScalar, int _mRow, int _mCol>
        Matrix(const Matrix<_mScalar, _mRow, _mCol> &m, _Scalar val = 0)
        {

            if (_mRow * _mCol == 0)
                return;
            data = (_Scalar *)malloc((_Rows * _Cols + _Rows * _Cols % 4) * sizeof(_Scalar));
            if (_Rows * _Cols >= _mCol * _mRow)
            {
                for (int i = 0; i < _mCol * _mRow; i++)
                    data[i] = (_Scalar)m[i];
                for (int i = _mCol * _mRow; i < _Rows * _Cols; i++)
                    data[i] = val;
            }
            else
                for (int i = 0; i < _Rows * _Cols; i++)
                    data[i] = (_Scalar)m[i];
        }
        ~Matrix()
        {

            free(this->data);
        }

        Matrix(std::initializer_list<_Scalar> values)
        {
            assert(values.size() == _Rows * _Cols);
            this->data = (_Scalar *)malloc((_Rows * _Cols + _Rows * _Cols % 4) * sizeof(_Scalar));
            auto it = values.begin();
            for (int i = 0; i < _Rows * _Cols; i++, it++)
                this->data[i] = *it;
        }

        void _impl_args(int index, const _Scalar &t)
        {
            this->data[index] = t;
        }

        template <typename... Args>
        void _impl_args(int index, const _Scalar &t, const Args &...rest)
        {
            this->data[index] = t;
            _impl_args(index + 1, rest...);
        }
        template <typename... Args>
        Matrix(const _Scalar &t, const Args &...rest)
        {
            this->data = (_Scalar *)malloc((_Rows * _Cols + _Rows * _Cols % 4) * sizeof...(rest) * sizeof(_Scalar));
            this->data[0] = t;
            _impl_args(1, rest...);
        }

        std::tuple<int, int> getShape()
        {
            return std::tuple<int, int>(row, col);
        }

        Matrix<_Scalar, _Cols, _Rows> T()
        {
            Matrix<_Scalar, _Cols, _Rows> m;
            for (int r = 0; r < _Cols; r++)
                for (int c = 0; c < _Rows; c++)
                    m[r * _Rows + c] = this->data[c * _Cols + r];

            return m;
        }

        Matrix operator+(const Matrix &b)
        {
            Matrix<_Scalar, _Rows, _Cols> m;
            for (int x = 0; x < _Rows * _Cols; x++)
                m[x] = this->data[x] + b[x];
            return m;
        }

        void operator+=(const Matrix &b)
        {
            for (int x = 0; x < _Rows * _Cols; x++)
                this->data[x] += b[x];
        }

        Matrix operator-(const Matrix &b)
        {
            Matrix<_Scalar, _Rows, _Cols> m;
            for (int x = 0; x < _Rows * _Cols; x++)
                m[x] = this->data[x] - b[x];
            return m;
        }
        Matrix operator*(const _Scalar &b)
        {
            Matrix<_Scalar, _Rows, _Cols> m;
            for (int x = 0; x < _Rows * _Cols; x++)
                m[x] = this->data[x] * b;
            return m;
        }
        Matrix operator*(const _Scalar &&b)
        {
            Matrix<_Scalar, _Rows, _Cols> m;
            for (int x = 0; x < _Rows * _Cols; x++)
                m[x] = this->data[x] * b;
            return m;
        }
        Matrix operator/(const _Scalar &b)
        {
            Matrix<_Scalar, _Rows, _Cols> m;
            for (int x = 0; x < _Rows * _Cols; x++)
                m[x] = this->data[x] / b;
            return m;
        }
        Matrix operator+(const _Scalar &b)
        {
            Matrix<_Scalar, _Rows, _Cols> m;
            for (int x = 0; x < _Rows * _Cols; x++)
                m[x] = this->data[x] + b;
            return m;
        }
        Matrix operator-(const _Scalar &b)
        {
            Matrix<_Scalar, _Rows, _Cols> m;
            for (int x = 0; x < _Rows * _Cols; x++)
                m[x] = this->data[x] - b;
            return m;
        }
        void operator=(const Matrix &b)
        {
            if (this->data)
                free(this->data);
            this->data = (_Scalar *)malloc((_Rows * _Cols + _Rows * _Cols % 4) * sizeof(_Scalar));
            for (int i = 0; i < _Rows * _Cols; i++)
                this->data[i] = b[i];
        }
        void operator=(Matrix &&b)
        {
            if (this->data)
                free(this->data);
            this->data = b.data;
            b.data = nullptr;
        }
        template <typename T1, int _bRow, int _bCol>
        Matrix<T1, _Rows, _bCol> operator*(const Matrix<T1, _bRow, _bCol> &b)
        {
            assert(_Cols == _bRow);
            Matrix<T1, _Rows, _bCol> res;
            for (int m = 0; m < _Rows; m++)
                for (int s = 0; s < _bCol; s++)
                {
                    res[m * _bCol + s] = 0;
                    for (int n = 0; n < _bRow; n++)
                        res[m * _bCol + s] += this->data[m * _Cols + n] * b[n * _bCol + s];
                }
            return res;
        }

        template <typename T1, int _bRow, int _bCol>
        Matrix<T1, _Rows, _bCol> operator^(const Matrix<T1, _bRow, _bCol> &b)
        {
            assert(_Rows == _bRow);
            Matrix<T1, _Cols, _bCol> res;
            for (int m = 0; m < _Cols; m++)
                for (int s = 0; s < _bCol; s++)
                {
                    T1 &x = res[m * _bCol + s];
                    x = 0;
                    for (int n = 0; n < _bRow; n++)
                        x += this->data[n * _Cols + m] * b[n * _bCol + s];
                }
            return res;
        }

        inline _Scalar &operator[](int i) const
        {
            return this->data[i];
        }

        friend std::ostream &operator<<(std::ostream &output,
                                        const Matrix &D)
        {
            for (int x = 0; x < _Rows; x++)
            {
                for (int y = 0; y < _Cols; y++)
                    output << D[x * _Cols + y] << ",";
                output << std::endl;
            }
            return output;
        }
    };

    using Vector3d = Matrix<double, 3, 1>;
    using Vector4d = Matrix<double, 4, 1>;
    using Vector6d = Matrix<double, 6, 1>;
    using Point2d = Matrix<double, 2, 1>;
    using Point2i = Matrix<int, 2, 1>;
    using _T = Matrix<double, 4, 4>;
    using _R = Matrix<double, 3, 3>;

    template <typename T>
    Matrix<T, 3, 3> hat(const Matrix<T, 3, 1> &m)
    {
        Matrix<T, 3, 3> hat_m;
        hat_m[1] = (-1) * m[2];
        hat_m[2] = m[1];
        hat_m[3] = m[2];
        hat_m[5] = (-1) * m[0];
        hat_m[6] = (-1) * m[1];
        hat_m[7] = m[0];
        return hat_m;
    }
    template <typename T>
    Matrix<T, 3, 1> vee(const Matrix<T, 3, 3> &m)
    {
        Matrix<T, 3, 1> vee_m;
        vee_m[0] = m[7];
        vee_m[1] = m[2];
        vee_m[2] = m[3];
        return vee_m;
    }

    template <typename T>
    Matrix<T, 3, 1> cross(const Matrix<T, 3, 1> &a, const Matrix<T, 3, 1> &b)
    {
        return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }

    template <typename T>
    T dot(const Matrix<T, 3, 1> &a, const Matrix<T, 3, 1> &b)
    {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    template <typename T, int L>
    Matrix<T, L, L> eye()
    {
        Matrix<T, L, L> m;
        for (int x = 0; x < L; x++)
            m[x * L + x] = 1;
        return m;
    }
    template <typename T1, int _aRow, int _Col, int _bRow>
    Matrix<T1, _aRow + _bRow, _Col> catRow(const Matrix<T1, _aRow, _Col> &a, const Matrix<T1, _bRow, _Col> &b)
    {
        Matrix<T1, _aRow + _bRow, _Col> m(a);
        for (int x = _aRow * _Col; x < _aRow * _Col + _bRow * _Col; x++)
            m[x] = b[x - _aRow * _Col];
        return m;
    }
    template <typename T1, int _Row, int _aCol, int _bCol>
    Matrix<T1, _Row, _aCol + _bCol> catCol(const Matrix<T1, _Row, _aCol> &a, const Matrix<T1, _Row, _bCol> &b)
    {
        Matrix<T1, _Row, _aCol + _bCol> m;
        for (int i = 0; i < _Row; i++)
            for (int j = 0; j < _aCol; j++)
                m[i * (_aCol + _bCol) + j] = a[i * _aCol + j];

        for (int i = 0; i < _Row; i++)
            for (int j = _aCol; j < _aCol + _bCol; j++)
                m[i * (_aCol + _bCol) + j] = b[i * _bCol + j - _aCol];
        return m;
    }

    struct pixel
    {
        Vector3d color;
        Point2i pos;
    };

    inline Matrix<double, 6, 1> operator*(Matrix<double, 6, 6> &a, Matrix<double, 6, 1> &b)
    {
        Matrix<double, 6, 1> res;
        for (int m = 0; m < 6; m++)
        {
            const int base = m * 6;
            res[m] = a[base + 0] * b[0] +
                     a[base + 1] * b[1] +
                     a[base + 2] * b[2] +
                     a[base + 3] * b[3] +
                     a[base + 4] * b[4] +
                     a[base + 5] * b[5];
        }
        return res;
    }

    inline Matrix<double, 1, 1> operator*(Matrix<double, 1, 6> &a, Matrix<double, 6, 1> &b)
    {
        return a[0] * b[0] +
               a[1] * b[1] +
               a[2] * b[2] +
               a[3] * b[3] +
               a[4] * b[4] +
               a[5] * b[5];
    }

    inline Matrix<double, 1, 1> operator*(Matrix<double, 6, 1> &a, Matrix<double, 6, 1> &b)
    {
        return a[0] * b[0] +
               a[1] * b[1] +
               a[2] * b[2] +
               a[3] * b[3] +
               a[4] * b[4] +
               a[5] * b[5];
    }

    inline Matrix<double, 6, 1> operator+(Matrix<double, 6, 1> &a, Matrix<double, 6, 1> &b)
    {
        double *ps = (double*)malloc(8*sizeof(double));
        const double *pa = a.data;
        const double *pb = b.data;

        __m256d va0 = _mm256_loadu_pd(pa);
        __m256d vb0 = _mm256_loadu_pd(pb);

        __m256d vr0 = _mm256_add_pd(va0, vb0);

        _mm256_storeu_pd(ps, vr0);

        va0 = _mm256_loadu_pd(pa+4);
        vb0 = _mm256_loadu_pd(pb+4);

        vr0 = _mm256_add_pd(va0, vb0);

        _mm256_storeu_pd(ps+4, vr0);

        return ps;
    }

    inline Matrix<double, 6, 1> operator+(Matrix<double, 6, 1> &a, Matrix<double, 6, 1> &&b)
    {

        double *pa = a.data;
        double *pb = b.data;

        __m256d va0 = _mm256_loadu_pd(pa);
        __m256d vb0 = _mm256_loadu_pd(pb);

        __m256d vr0 = _mm256_add_pd(va0, vb0);

        _mm256_storeu_pd(pb, vr0);

        va0 = _mm256_loadu_pd(pa+4);
        vb0 = _mm256_loadu_pd(pb+4);

        vr0 = _mm256_add_pd(va0, vb0);

        _mm256_storeu_pd(pb+4, vr0);

        return b;
    }

    inline Matrix<double, 6, 6> operator*(const Matrix<double, 6, 6> &a, const Matrix<double, 6, 6> &b)
    {
        Matrix<double, 6, 6> res;

        for (int i = 0; i < 6; ++i)
        {
            const int ai0 = i * 6;
            for (int j = 0; j < 6; ++j)
            {
                res[ai0 + j] = a[ai0 + 0] * b[0 * 6 + j] +
                               a[ai0 + 1] * b[1 * 6 + j] +
                               a[ai0 + 2] * b[2 * 6 + j] +
                               a[ai0 + 3] * b[3 * 6 + j] +
                               a[ai0 + 4] * b[4 * 6 + j] +
                               a[ai0 + 5] * b[5 * 6 + j];
            }
        }

        return res;
    }

    inline Matrix<double, 3, 3> operator%(const Matrix<double, 3, 3> &a, const Matrix<double, 3, 1> &b)
    {
        const double b0 = b[0];
        const double b1 = b[1];
        const double b2 = b[2];
        return {
            a[1] * b2 - a[2] * b1,
            -a[0] * b2 + a[2] * b0,
            a[0] * b1 - a[1] * b0,
            a[4] * b2 - a[5] * b1,
            -a[3] * b2 + a[5] * b0,
            a[3] * b1 - a[4] * b0,
            a[7] * b2 - a[8] * b1,
            -a[6] * b2 + a[8] * b0,
            a[6] * b1 - a[7] * b0,
        };
    }

    inline Matrix<double, 3, 3> operator%(const Matrix<double, 3, 1> &a, const Matrix<double, 3, 1> &b)
    {
        const double x22 = -a[2] * b[2];
        const double x21 = a[2] * b[1];
        const double x20 = a[2] * b[0];
        const double x11 = -a[1] * b[1];
        const double x10 = a[1] * b[0];
        const double x00 = -a[0] * b[0];

        return {
            x22 + x11, x10, x20,
            x10, x22 + x00, x21,
            x20, x21, x11 + x00};
    }

}
