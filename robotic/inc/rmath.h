#pragma once
#include <matrix.h>
#include <cmath>

namespace Engine
{
    using M66 = Matrix<double, 6, 6>;
    inline _R rx(double theta)
    {
        return _R{1, 0, 0,
                  0, std::cos(theta), std::sin(theta),
                  0, -std::sin(theta), std::cos(theta)};
    }
    inline _R ry(double theta)
    {
        return _R{std::cos(theta), 0, -std::sin(theta),
                  0, 1, 0,
                  std::sin(theta), 0, std::cos(theta)};
    }
    inline _R rz(double theta)
    {
        return _R{std::cos(theta), std::sin(theta), 0,
                  -std::sin(theta), std::cos(theta), 0,
                  0, 0, 1};
    }

    template <typename T>
    M66 rot(T &&E)
    {
        return M66{E[0], E[1], E[2], 0, 0, 0,
                   E[3], E[4], E[5], 0, 0, 0,
                   E[6], E[7], E[8], 0, 0, 0,
                   0, 0, 0, E[0], E[1], E[2],
                   0, 0, 0, E[3], E[4], E[5],
                   0, 0, 0, E[6], E[7], E[8]};
    }

    template <typename T>
    M66 xlt(T &&t)
    {
        return M66{1, 0, 0, 0, 0, 0,
                   0, 1, 0, 0, 0, 0,
                   0, 0, 1, 0, 0, 0,
                   0, t[2], -t[1], 1, 0, 0,
                   -t[2], 0, t[0], 0, 1, 0,
                   t[1], -t[0], 0, 0, 0, 1};
    }

    template <typename T>
    inline void inv_plx(T &&x, _R &E, Vector3d &p)
    {
        _R r{x[0], x[1], x[2], x[6], x[7], x[8], x[12], x[13], x[14]};
        _R pxr{x[18], x[19], x[20], x[24], x[25], x[26], x[30], x[31], x[32]};
        E = r;
        p = vee(pxr * r.T());
    }

    template <typename T1,typename T2>
    inline Matrix<double, 6, 6> plx(T1 &&E, T2 &&p)
    {
        return catRow(catCol(E, _R()), catCol(E * hat(p) * (-1), E));
    }

    template <typename T>
    inline Matrix<double, 6, 6> crm(T &&v)
    {
        return catRow(catCol(hat(Vector3d{v[0], v[1], v[2]}), _R()), 
                catCol(hat(Vector3d{v[3], v[4], v[5]}), hat(Vector3d{v[0], v[1], v[2]})));
    }

    template <typename T>
    inline Matrix<double, 6, 6> crf(T &&v)
    {
        return  crm(v).T()*(-1);
    }

    
}