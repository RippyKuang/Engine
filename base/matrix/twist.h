#pragma once
#include <matrix.h>

namespace Engine
{

    using ad_se3 = Matrix<double, 6, 6>;

    inline ad_se3 adjoint(const _T &t)
    {
        Vector3d p{t[3], t[3 + 1 * 4], t[3 + 2 * 4]};
        _R r{t[0], t[1], t[2], t[4], t[5], t[6], t[8], t[9], t[10]};
        return catRow(catCol(r, _R()), catCol(hat(p) * r, r));
    }

    inline _T inv_ad(ad_se3 x)
    {
        _R r{x[0], x[1], x[2], x[6], x[7], x[8], x[12], x[13], x[14]};
        _R pxr{x[18], x[19], x[20], x[24], x[25], x[26], x[30], x[31], x[32]};
        return catRow(catCol(r, vee(pxr*r.T())), catCol(Vector3d().T(), EYE(1)));
    }

    class Twist : public Matrix<double, 6, 1>
    {
    public:
        Twist(double v = 0)
        {
            for (int i = 0; i < 6; i++)
                (*this)[i] = v;
        }

        template <typename T, typename = typename std::enable_if<std::is_base_of<Matrix, T>::value>::type>
        Twist(T &&m) : T(std::forward<T>(m))
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

    template <typename T1, typename T2>
    inline Twist bracket(T1 &&v1, T2 &&v2)
    {
        Vector3d v1_w{v1[0], v1[1], v1[2]};
        Vector3d v1_v{v1[3], v1[4], v1[5]};
        Vector3d v2_w{v2[0], v2[1], v2[2]};
        Vector3d v2_v{v2[3], v2[4], v2[5]};

        return Twist(cross(v1_w, v2_w), cross(v1_w, v2_v) + cross(v1_v, v2_w));
    }

}