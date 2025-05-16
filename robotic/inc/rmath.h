#pragma once
#include <matrix.h>
#include <cmath>

namespace Engine
{
    using M66 = Matrix<double, 6, 6>;
    inline _R rx(double theta)
    {
        return _R{1, 0, 0,
                  0, std::cos(theta), -std::sin(theta),
                  0, std::sin(theta), std::cos(theta)};
    }
    inline _R ry(double theta)
    {
        return _R{std::cos(theta), 0, std::sin(theta),
                  0, 1, 0,
                  -std::sin(theta), 0, std::cos(theta)};
    }
    inline _R rz(double theta)
    {
        return _R{std::cos(theta), -std::sin(theta), 0,
                  std::sin(theta), std::cos(theta), 0,
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
}