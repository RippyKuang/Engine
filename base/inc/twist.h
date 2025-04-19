#pragma once
#include <matrix.h>

namespace Engine
{

    using ad_se3 = Matrix<double, 6, 6>;

    inline  ad_se3 adjoint(const _T& t)
    {   
        Vector3d p{t[3], t[3 + 1 * 4], t[3 + 2 * 4]};
        _R r{t[0], t[1], t[2], t[4], t[5], t[6], t[8], t[9], t[10]};
        return catRow(catCol(r, _R()), catCol(hat(p) * r, r));
    }

    class Twist : public Matrix<double, 6, 1>
    {
    public:
        Twist(double v = 0)
        {
            for (int i = 0; i < 6; i++)
                (*this)[i] = v;
        }

        Twist(const Matrix<double, 6, 1> &m) : Matrix<double, 6, 1>(m)
        {
        }
        Twist(const Matrix<double, 3, 1> &v, const Matrix<double, 3, 1> &w)
        {
            (*this)[0] = v[0];
            (*this)[1] = v[1];
            (*this)[2] = v[2];
            (*this)[3] = w[0];
            (*this)[4] = w[1];
            (*this)[5] = w[2];
        }
    };

}