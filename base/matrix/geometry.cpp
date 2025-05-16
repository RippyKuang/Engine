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

    _R rpy2rot(Vector3d rpy)
    {
        double c1 = std::cos(rpy[0]);
        double s1 = std::sin(rpy[0]);
        double c2 = std::cos(rpy[1]);
        double s2 = std::sin(rpy[1]);
        double c3 = std::cos(rpy[2]);
        double s3 = std::sin(rpy[2]);
        return _R{c2 * c3, c1 * s3 + s1 * s2 * c3, s1 * s3 - c1 * s2 * c3,
                  -c2 * s3, c1 * c3 - s1 * s2 * s3, s1 * c3 + c1 * s2 * s3,
                  s2, -s1 * c2, c1 * c2};
    }

    _T inv(_T&& t)
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

    _T inv(_T& t)
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

  
    Vector3d& norm(Vector3d &v)
    {
        double norm = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (norm == 0)
            return v;
        for (int i = 0; i < 3; i++)
            v[i] /= norm;
        return v;
    }

    std::vector<Vector3d> to_3d(std::vector<Vector4d> &corners)
    {
        std::vector<Vector3d> _3d;
        for (auto corner : corners)
            _3d.push_back(Vector3d(corner));
        return _3d;
    }

    std::vector<Vector3d> to_3d(std::vector<Vector4d> &&corners)
    {
        std::vector<Vector3d> _3d;
        for (auto corner : corners)
            _3d.push_back(Vector3d(corner));
        return _3d;
    }

   
    void getRT(_T& t, _R &r, Vector3d &tvec)
    {
        for (int x = 0; x < 3; x++)
            for (int y = 0; y < 3; y++)
                r[x * 3 + y] = t[x * 4 + y];
        for (int x = 0; x < 3; x++)
            tvec[x] = t[x * 4 + 3];
    }


}