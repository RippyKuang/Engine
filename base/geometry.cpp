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
        return getTransformMat(r.T(), r.T() * vec * (-1));
    }
    _R AngleAxis::toRotationMat()
    {
        return EYE(3) * std::cos(angle) + axis * axis.T() * (1 - std::cos(angle)) + hat(axis) * std::sin(angle);
    }

        bool isPointInTriangle(Vector3d &P, Vector3d &A, Vector3d &B, Vector3d &C)
    {
        Vector3d v0 = C - A, v1 = B - A, v2 = P - A;

        double d00 = dot(v0,v0);
        double d01 = dot(v0,v1);
        double d11 = dot(v1,v1);
        double d20 = dot(v2,v0);
        double d21 = dot(v2,v1);

        double denom = d00 * d11 - d01 * d01;
        double v = (d11 * d20 - d01 * d21) / denom;
        double w = (d00 * d21 - d01 * d20) / denom;
        double u = 1.0 - v - w;

        return (u >= 0) && (v >= 0) && (w >= 0);
    }

    bool doesLineIntersectTriangle(Vector3d &P, Vector3d &A, Vector3d &B, Vector3d &C)
    {
        Vector3d N = cross(B - A, C - A);
        double denom = dot(N, P);
        if (denom == 0)
            return false; 

        double t = dot(N,A) / denom;
        if (t < 0)
            return false; 

        Vector3d intersection = Vector3d{P[0] * t, P[1] * t, P[2] * t};
        return isPointInTriangle(intersection, A, B, C);
    }

    std::vector<Vector4d> remove_hidden(const std::vector<Vector4d> &corners)
    {
        std::vector<Vector4d> return_corners;

        Vector3d c0 = Vector3d(corners[0]);
        Vector3d c1 = Vector3d(corners[1]);
        Vector3d c2 = Vector3d(corners[2]);
        Vector3d c3 = Vector3d(corners[3]);
        Vector3d c4 = Vector3d(corners[4]);
        Vector3d c5 = Vector3d(corners[5]);
        Vector3d c6 = Vector3d(corners[6]);
        Vector3d c7 = Vector3d(corners[7]);

        CHECK_HIDDEN(0, 1, 2, 4); 
        CHECK_HIDDEN(1, 0, 3, 5);
        CHECK_HIDDEN(2, 3, 6, 0); 
        CHECK_HIDDEN(3, 7, 2, 1);
        CHECK_HIDDEN(4, 5, 6, 0);
        CHECK_HIDDEN(5, 7, 1, 4);
        CHECK_HIDDEN(6, 7, 4, 2);
        CHECK_HIDDEN(7, 5, 6, 3);
        
        return return_corners;
    }


}