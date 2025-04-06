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

        double d00 = dot(v0, v0);
        double d01 = dot(v0, v1);
        double d11 = dot(v1, v1);
        double d20 = dot(v2, v0);
        double d21 = dot(v2, v1);

        double denom = d00 * d11 - d01 * d01;
        double v = (d11 * d20 - d01 * d21) / denom;
        double w = (d00 * d21 - d01 * d20) / denom;
        double u = 1.0 - v - w;

        return (u >= 0) && (v >= 0) && (w >= 0);
    }

    Vector3d intersectLinePlane(Vector3d &P, Vector3d &A, Vector3d &B, Vector3d &C)
    {
        Vector3d N = cross(B - A, C - A);
        double denom = dot(N, P);
        if (denom == 0)
            return Vector3d{_FALSE, _FALSE, _FALSE};

        double t = dot(N, A) / denom;
        if (t <= 0 || t >= 1)
            return Vector3d{_FALSE, _FALSE, _FALSE};

        Vector3d intersection = Vector3d{P[0] * t, P[1] * t, P[2] * t};
        return intersection;
    }

    bool LineIntersectTriangle(Vector3d &P, Vector3d &A, Vector3d &B, Vector3d &C)
    {
        Vector3d intersection = intersectLinePlane(P, A, B, C);
        if (intersection[0] == _FALSE)
            return false;
        return isPointInTriangle(intersection, A, B, C);
    }

    bool LineIntersectQuadrilateral(Vector3d &P, Vector3d &A, Vector3d &B, Vector3d &C, Vector3d &D)
    {
        Vector3d intersection = intersectLinePlane(P, A, B, C);
        if (intersection[0] == _FALSE)
            return false;
        return isPointInTriangle(intersection, A, B, C) || isPointInTriangle(intersection, A, C, D);
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

    void remove_self_hidden(std::vector<Vector3d> &corners, std::vector<bool> &visible)
    {

#define CHECK_SELF_HIDDEN(p, pa, pb, pc)                                              \
    do                                                                                \
    {                                                                                 \
        if (LineIntersectTriangle(corners[p], corners[pa], corners[pb], corners[pc])) \
            visible[p] = false;                                                       \
    } while (0)

        CHECK_SELF_HIDDEN(0, 1, 2, 4);
        CHECK_SELF_HIDDEN(1, 0, 3, 5);
        CHECK_SELF_HIDDEN(2, 3, 6, 0);
        CHECK_SELF_HIDDEN(3, 7, 2, 1);
        CHECK_SELF_HIDDEN(4, 5, 6, 0);
        CHECK_SELF_HIDDEN(5, 7, 1, 4);
        CHECK_SELF_HIDDEN(6, 7, 4, 2);
        CHECK_SELF_HIDDEN(7, 5, 6, 3);
#undef CHECK_SELF_HIDDEN
    }

    void remove_inter_hidden(std::vector<Vector3d> &corners, std::vector<Vector3d> &item, std::vector<bool> &visible)
    {
        for (int i = 0; i < corners.size(); i++)
        {
            Vector3d p = corners[i];

            if (LineIntersectQuadrilateral(p, item[0], item[1], item[5], item[4]) ||
                LineIntersectQuadrilateral(p, item[0], item[1], item[3], item[2]) ||
                LineIntersectQuadrilateral(p, item[0], item[4], item[6], item[2]) ||
                LineIntersectQuadrilateral(p, item[2], item[6], item[7], item[3]) ||
                LineIntersectQuadrilateral(p, item[4], item[6], item[7], item[5]) ||
                LineIntersectQuadrilateral(p, item[1], item[3], item[7], item[5]))
                visible[i] = false;
        }
    }

    Quaternion rotationMatrixToQuaternion(const _R &m)
    {
        Quaternion q;
        double trace = m[0] + m[1 * 3 + 1] + m[2 * 3 + 2];

        if (trace > 0.0)
        {
            double s = 0.5 / std::sqrt(trace + 1.0);
            q.w = 0.25 / s;
            q.x = (m[2 * 3 + 1] - m[1 * 3 + 2]) * s;
            q.y = (m[0 * 3 + 2] - m[2 * 3 + 0]) * s;
            q.z = (m[1 * 3 + 0] - m[0 * 3 + 1]) * s;
        }
        else
        {
            if (m[0 * 3 + 0] > m[1 * 3 + 1] && m[0 * 3 + 0] > m[2 * 3 + 2])
            {
                double s = 2.0 * std::sqrt(1.0 + m[0 * 3 + 0] - m[1 * 3 + 1] - m[2 * 3 + 2]);
                q.w = (m[2 * 3 + 1] - m[1 * 3 + 2]) / s;
                q.x = 0.25 * s;
                q.y = (m[0 * 3 + 1] + m[1 * 3 + 0]) / s;
                q.z = (m[0 * 3 + 2] + m[2 * 3 + 0]) / s;
            }
            else if (m[1 * 3 + 1] > m[2 * 3 + 2])
            {
                double s = 2.0 * std::sqrt(1.0 + m[1 * 3 + 1] - m[0 * 3 + 0] - m[2 * 3 + 2]);
                q.w = (m[0 * 3 + 2] - m[2 * 3 + 0]) / s;
                q.x = (m[0 * 3 + 1] + m[1 * 3 + 0]) / s;
                q.y = 0.25 * s;
                q.z = (m[1 * 3 + 2] + m[2 * 3 + 1]) / s;
            }
            else
            {
                double s = 2.0 * std::sqrt(1.0 + m[2 * 3 + 2] - m[0 * 3 + 0] - m[1 * 3 + 1]);
                q.w = (m[1 * 3 + 0] - m[0 * 3 + 1]) / s;
                q.x = (m[0 * 3 + 2] + m[2 * 3 + 0]) / s;
                q.y = (m[1 * 3 + 2] + m[2 * 3 + 1]) / s;
                q.z = 0.25 * s;
            }
        }
        return q;
    }
    _R Quaternion::toRotationMat()
    {
        _R m;
        double ww = w * w;
        double xx = x * x;
        double yy = y * y;
        double zz = z * z;

        m[0 * 3 + 0] = ww + xx - yy - zz;
        m[0 * 3 + 1] = 2 * (x * y - w * z);
        m[0 * 3 + 2] = 2 * (x * z + w * y);

        m[1 * 3 + 0] = 2 * (x * y + w * z);
        m[1 * 3 + 1] = ww - xx + yy - zz;
        m[1 * 3 + 2] = 2 * (y * z - w * x);

        m[2 * 3 + 0] = 2 * (x * z - w * y);
        m[2 * 3 + 1] = 2 * (y * z + w * x);
        m[2 * 3 + 2] = ww - xx - yy + zz;

        return m;
    }
    void Quaternion::norm()
    {
        double norm = std::sqrt(w * w + x * x + y * y + z * z);
        if (norm == 0)
            return;
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    }

}