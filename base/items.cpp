#include <items.h>

namespace Engine
{
    void Link::transform(_T t)
    {
        for (auto it = this->corners.begin(); it != this->corners.end(); it++)
            *it = t * (*it);
    }

    Link::Link(std::vector<Vector3d> _corners)
    {
        for (auto corner : _corners)
            this->corners.push_back(Vector4d(corner, 1));
    }

    Cube::Cube(Vector3d box, Vector3d rpy, Vector3d xyz)
    {
        double x = box[0];
        double y = box[1];
        double z = box[2];
        corners.push_back(Vector4d{+x / 2, -y / 2, +z / 2, 1});
        corners.push_back(Vector4d{+x / 2, -y / 2, -z / 2, 1});
        corners.push_back(Vector4d{+x / 2, +y / 2, +z / 2, 1});
        corners.push_back(Vector4d{+x / 2, +y / 2, -z / 2, 1});

        corners.push_back(Vector4d{-x / 2, -y / 2, +z / 2, 1});
        corners.push_back(Vector4d{-x / 2, -y / 2, -z / 2, 1});
        corners.push_back(Vector4d{-x / 2, +y / 2, +z / 2, 1});
        corners.push_back(Vector4d{-x / 2, +y / 2, -z / 2, 1});
        init_pose = getTransformMat(rpy2rot(rpy), xyz);
        transform(init_pose);
    }

}
