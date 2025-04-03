#include <items.h>

namespace Engine
{
    void Item::transform(_T t)
    {
        for (auto it = this->corners.begin(); it != this->corners.end(); it++)
            *it = t * (*it);
    }

    Item::Item(std::vector<Vector3d> _corners)
    {
        for (auto corner : _corners)
            this->corners.push_back(Vector4d(corner, 1));
    }
    Cube::Cube(Vector3d center, double x, double y, double z)
    {
        corners.push_back(Vector4d{center[0] + x / 2, center[1] - y / 2, center[2] + z / 2, 1});
        corners.push_back(Vector4d{center[0] + x / 2, center[1] - y / 2, center[2] - z / 2, 1});
        corners.push_back(Vector4d{center[0] + x / 2, center[1] + y / 2, center[2] + z / 2, 1});
        corners.push_back(Vector4d{center[0] + x / 2, center[1] + y / 2, center[2] - z / 2, 1});

        corners.push_back(Vector4d{center[0] - x / 2, center[1] - y / 2, center[2] + z / 2, 1});
        corners.push_back(Vector4d{center[0] - x / 2, center[1] - y / 2, center[2] - z / 2, 1});
        corners.push_back(Vector4d{center[0] - x / 2, center[1] + y / 2, center[2] + z / 2, 1});
        corners.push_back(Vector4d{center[0] - x / 2, center[1] + y / 2, center[2] - z / 2, 1});
        init_pose = getTransformMat(EYE(3), center);
    }
}
