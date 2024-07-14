#include <geometry.h>

namespace Engine
{
    _T getTransformMat(AngleAxis aa, Vector3d t)
    {
        return catRow(catCol(aa.toRotationMat(), t), catCol(Vector3d().T(), EYE(1)));
    }
    _T getTransformMat(_R aa, Vector3d t)
    {
        return catRow(catCol(aa, t), catCol(Vector3d().T(), EYE(1)));
    }

    _T inv(_T t)
    {
        _R r;
        Vector3d vec;
        for (int x = 0; x < 3; x++)
            for (int y = 0; y < 3; y++)
                r[x * 3 + y] = t[x * 4 + y];
        for (int x = 0; x < 3; x++)
            vec[x] = t[x * 4 + 3];
        return getTransformMat(r.T(), r.T()*vec*(-1));
    }
    _R AngleAxis::toRotationMat()
    {
        return EYE(3) * std::cos(angle) + axis * axis.T() * (1 - std::cos(angle)) + hat(axis) * std::sin(angle);
    }

}