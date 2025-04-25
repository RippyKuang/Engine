#include <matrix.h>
#include <math.h>
#include <marcos.h>

namespace Engine
{

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
    _R rpy2rot(Vector3d rpy);
    std::vector<Vector3d> to_3d(std::vector<Vector4d> &);
    std::vector<Vector3d> to_3d(std::vector<Vector4d> &&);
    void remove_self_hidden(std::vector<Vector3d> &, std::vector<bool> &);
    void remove_inter_hidden(std::vector<Vector3d> &, std::vector<Vector3d> &, std::vector<bool> &);
}