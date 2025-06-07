#include "matrix.h"

namespace Engine
{

    struct obb_box
    {
        Vector3d center;
        Vector3d e;
        _R pose;
    };

    struct contact_results
    {
        Vector3d normal;
        Vector3d points;
        double depth;
    };

    int obb_Intersection(const obb_box &box1, const obb_box &box2, int maxc, std::vector<contact_results> &output, int &return_code);
}