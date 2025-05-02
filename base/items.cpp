#include <items.h>

namespace Engine
{
    void Link::transform(_T t)
    {
        this->mesh._transform(t);
    }

    void Link::mesh_transform(_T t)
    {
        this->mesh._transform(t);
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
        _R I = {mass * (y + z) / 12, 0, 0,
                0, mass * (x + z) / 12, 0,
                0, 0, mass * (x + y) / 12};
        this->inertia = adjoint(inv_pose).T() * (catRow(catCol(I,_R()),catCol(_R(), EYE(3) * mass ))) * adjoint(inv_pose);
    }

}
