#pragma once
#include <tuple>
#include <iostream>
#include <assert.h>
#include <initializer_list>
#include <cstring>
#include <vector>

#define EYE(x) Engine::eye<double, x>()

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
        Matrix(_Scalar *p) : data(p)
        {
        }
        Matrix(_Scalar val = 0)
        {
            data = (_Scalar *)malloc(_Rows * _Cols * sizeof(_Scalar));
            for (int i = 0; i < _Rows * _Cols; i++)
                data[i] = val;
        }

        Matrix(const Matrix<_Scalar, _Rows, _Cols> &m)
        {

            data = (_Scalar *)malloc(_Rows * _Cols * sizeof(_Scalar));
            for (int i = 0; i < _Rows * _Cols; i++)
                data[i] = m[i];
        }
        template <typename _mScalar, int _mRow, int _mCol>
        Matrix(const Matrix<_mScalar, _mRow, _mCol> &m, _Scalar val = 0)
        {
            if (_mRow * _mCol == 0)
                return;
            data = (_Scalar *)malloc(_Rows * _Cols * sizeof(_Scalar));
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
            this->data = (_Scalar *)malloc(_Rows * _Cols * sizeof(_Scalar));
            auto it = values.begin();
            for (int i = 0; i < _Rows * _Cols; i++, it++)
                this->data[i] = *it;
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
        Matrix &operator=(const Matrix &b)
        {
            this->data = (_Scalar *)malloc(_Rows * _Cols * sizeof(_Scalar));
            for (int i = 0; i < _Rows * _Cols; i++)
                this->data[i] = b[i];
            return *this;
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

        _Scalar &operator[](int i) const
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
    Matrix<T, 3, 1> cross(const Matrix<T, 3, 1> &a, const Matrix<T, 3, 1> &b)
    {
        return Matrix<T, 3, 1>{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
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
}