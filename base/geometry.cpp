#include <geometry.h>

namespace Engine
{
    _T getTransformMat(AngleAxis aa,Vector3d t)
    {
        return  catRow(catCol(aa.toRotationMat(), t), catCol(Vector3d().T(), EYE(1)));
    }
    _T getTransformMat(_R aa,Vector3d t)
    {
        return  catRow(catCol(aa, t), catCol(Vector3d().T(), EYE(1)));
    }
    _R AngleAxis::toRotationMat()
    {
        return EYE(3) * std::cos(angle) + axis * axis.T() * (1 - std::cos(angle)) + hat(axis) * std::sin(angle);
    }

}