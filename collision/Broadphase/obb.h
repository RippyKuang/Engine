#include "matrix.h"

namespace Engine
{
    struct obb_box
    {
        Vector3d center;
        Vector3d e;
        _R pose;
    };

    int obb_Intersection(const obb_box &box1, const obb_box &box2, Vector3d &normal, double *depth);
}