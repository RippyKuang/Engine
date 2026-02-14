#include "items.h"

namespace Engine
{
    void Link::transform(_T t)
    {
        this->mesh._transform(t);
    }

    void Link::set_name(std::string name)
    {
        this->name = name;
    }

    std::vector<Vector4d> &Link::get_corners()
    {
        return this->mesh.vertices;
    }

    std::vector<Vector3f> Link::fget_corners()
    {
        std::vector<Vector3f> res;
        for (auto vert : this->mesh.vertices)
        {
            res.push_back(vert);
        }
        return res;
    }

    Cube::Cube(Vector3d box, Vector3d rpy, Vector3d xyz, double mass) : Link(cube_mesh(box))
    {
        this->box = box;
        double x = box[0];
        double y = box[1];
        double z = box[2];
        this->mass = mass;
        this->center = xyz;

        _T init_pose = getTransformMat(rpy2rot(rpy), xyz);
        transform(init_pose);
        j2w = adjoint(inv(init_pose));
        _R I = {this->mass * (y * y + z * z) / 12, 0, 0,
                0, this->mass * (x * x + z * z) / 12, 0,
                0, 0, this->mass * (x * x + y * y) / 12};
        this->inertia = j2w.T() * (catRow(catCol(I, _R()), catCol(_R(), EYE(3) * this->mass))) * j2w;
    }

}
