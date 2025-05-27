#include <items.h>

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

    std::vector<Vector4d>& Link::get_corners()
    {
        return this->mesh.vertices;
    }

    Cube::Cube(Vector3d box, Vector3d rpy, Vector3d xyz, double mass):Link(cube_mesh(box))
    {
        double x = box[0];
        double y = box[1];
        double z = box[2];
        this->mass = mass;

        _T init_pose = getTransformMat(rpy2rot(rpy), xyz);
        transform(init_pose);
        _T inv_pose = inv(init_pose);
        _R I = {this->mass * (y*y + z*z) / 12, 0, 0,
                0, this->mass * (x*x + z*z) / 12, 0,
                0, 0, this->mass * (x*x + y*y) / 12};
        this->inertia = adjoint(inv_pose).T() * (catRow(catCol(I,_R()),catCol(_R(), EYE(3) * this->mass ))) * adjoint(inv_pose);
    }

}
