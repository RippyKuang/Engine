#pragma once
#include <matrix.h>
#include <cmath>

namespace Engine
{
    using M66 = Matrix<double, 6, 6>;
    inline _R rx(double theta)
    {
        return {1, 0, 0,
                0, std::cos(theta), std::sin(theta),
                0, -std::sin(theta), std::cos(theta)};
    }
    inline _R ry(double theta)
    {
        return {std::cos(theta), 0, -std::sin(theta),
                0, 1, 0,
                std::sin(theta), 0, std::cos(theta)};
    }
    inline _R rz(double theta)
    {
        return {std::cos(theta), std::sin(theta), 0,
                -std::sin(theta), std::cos(theta), 0,
                0, 0, 1};
    }

    template <typename T>
    inline M66 rot(T &&E)
    {
        return {E[0], E[1], E[2], 0, 0, 0,
                E[3], E[4], E[5], 0, 0, 0,
                E[6], E[7], E[8], 0, 0, 0,
                0, 0, 0, E[0], E[1], E[2],
                0, 0, 0, E[3], E[4], E[5],
                0, 0, 0, E[6], E[7], E[8]};
    }

    template <typename T>
    inline M66 xlt(T &&t)
    {
        return {1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0,
                0, t[2], -t[1], 1, 0, 0,
                -t[2], 0, t[0], 0, 1, 0,
                t[1], -t[0], 0, 0, 0, 1};
    }

    inline M66 rot_mul_xlt(const M66 &E, const M66 &t)
    {
        double e0 = E[0];
        double e1 = E[1];
        double e2 = E[2];
        double e3 = E[6];
        double e4 = E[7];
        double e5 = E[8];
        double e6 = E[12];
        double e7 = E[13];
        double e8 = E[14];

        double t0 = t[26];
        double t1 = t[30];
        double t2 = t[19];

        double a11 = -e1 * t2 + e2 * t1;
        double a12 = e0 * t2 - e2 * t0;
        double a13 = -e0 * t1 + e1 * t0;

        double a21 = -e4 * t2 + e5 * t1;
        double a22 = e3 * t2 - e5 * t0;
        double a23 = -e3 * t1 + e4 * t0;

        double a31 = -e7 * t2 + e8 * t1;
        double a32 = e6 * t2 - e8 * t0;
        double a33 = -e6 * t1 + e7 * t0;

        return {e0, e1, e2, 0, 0, 0,
                e3, e4, e5, 0, 0, 0,
                e6, e7, e8, 0, 0, 0,
                a11, a12, a13, e0, e1, e2,
                a21, a22, a23, e3, e4, e5,
                a31, a32, a33, e6, e7, e8};
    }

    template <typename T>
    inline void inv_plx(T &&x, _R &E, Vector3d &p)
    {
        E[0] = x[0];
        E[1] = x[1];
        E[2] = x[2];

        E[3] = x[6];
        E[4] = x[7];
        E[5] = x[8];

        E[6] = x[12];
        E[7] = x[13];
        E[8] = x[14];
        p[0] = x[20] * x[1] + x[26] * x[7] + x[32] * x[13];
        p[1] = x[18] * x[2] + x[24] * x[8] + x[30] * x[14];
        p[2] = x[19] * x[0] + x[26] * x[6] + x[32] * x[12];
    }

    template <typename T1, typename T2>
    inline Matrix<double, 6, 6> plx(T1 &&E, T2 &&p)
    {
        return catRow(catCol(E, _R()), catCol(E * hat(p) * (-1), E));
    }

    template <typename T>
    inline Matrix<double, 6, 6> crm(T &&v)
    {

        double v0 = v[0];
        double v1 = v[1];
        double v2 = v[2];
        double v3 = v[3];
        double v4 = v[4];
        double v5 = v[5];
        return {0, -v2, v1, 0, 0, 0,
                v2, 0, -v0, 0, 0, 0,
                -v1, v0, 0, 0, 0, 0,
                0, -v5, v4, 0, -v2, v1,
                v5, 0, -v3, v2, 0, -v0,
                -v4, v3, 0, -v1, v0, 0};
    }

    template <typename T>
    inline Matrix<double, 6, 6> crf(T &&v)
    {

        double v0 = v[0];
        double v1 = v[1];
        double v2 = v[2];
        double v3 = v[3];
        double v4 = v[4];
        double v5 = v[5];

        return {0, -v2, v1, 0, -v5, v4,
                v2, 0, -v0, v5, 0, -v3,
                -v1, v0, 0, -v4, v3, 0,
                0, 0, 0, 0, -v2, v1,
                0, 0, 0, v2, 0, -v0,
                0, 0, 0, -v1, v0, 0};
    }

    inline Matrix<double, 6, 6> XTIX(M66 &X, M66 &Ic)
    {

        _R X_E;
        Vector3d X_p;
        inv_plx(X, X_E, X_p);
        _R I{Ic[0], Ic[1], Ic[2], Ic[6], Ic[7], Ic[8], Ic[12], Ic[13], Ic[14]};
        Vector3d vh{Ic[16], Ic[5], Ic[9]};
        double m = Ic[21];
        Vector3d h = (X_E ^ vh) + X_p * m;
        _R i = (X_E ^ I) * X_E - h % X_p - (X_p) % (X_E ^ vh);
        double h0 = h[0];
        double h1 = h[1];
        double h2 = h[2];
        return {
            i[0], i[1], i[2], 0, -h2, h1,
            i[3], i[4], i[5], h2, 0, -h0,
            i[6], i[7], i[8], -h1, h0, 0,
            0, h2, -h1, m, 0, 0,
            -h2, 0, h0, 0, m, 0,
            h1, -h0, 0, 0, 0, m};
    }

}