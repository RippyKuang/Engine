#include <matrix.h>
#include <math.h>

namespace Engine
{
#define CHECK_HIDDEN(p, pa, pb, pc)                            \
    do                                                            \
    {                                                             \
        if (doesLineIntersectTriangle(c##p, c##pa, c##pb, c##pc)) \
            return_corners.push_back(Vector4d{-1, -1, -1, -1});   \
        else                                                      \
            return_corners.push_back(corners[p]);                 \
    } while (0)

    class AngleAxis : public Vector3d
    {
    private:
        double angle;
        Vector3d axis;

    public:
        AngleAxis(double _angle, Vector3d _axis) : Vector3d(_axis * _angle),
                                                   angle(_angle), axis(_axis)
        {
        }
        _R toRotationMat();
    };
    _T getTransformMat(AngleAxis aa, Vector3d t);
    _T getTransformMat(_R aa, Vector3d t);
    _T inv(_T t);
    std::vector<Vector4d> remove_hidden(const std::vector<Vector4d> &);
}