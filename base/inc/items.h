#pragma once
#include <geometry.h>
#include <vector>
#include <mesh.h>

namespace Engine
{
    class Link
    {
        friend class World;

        friend class Joint;

    protected:
        double mass;

        void mesh_transform(_T);
        std::vector<Vector4d> corners;
        Mesh mesh;

    public:
        Link(Mesh &m) : mesh(m) {};

        Link(Mesh &&t) : mesh(std::forward<Mesh>(t)) {};

        template <typename T, typename = typename std::enable_if<std::is_base_of<Link, T>::value>::type>
        Link(T &&t)
        {
            this->corners = std::move(t.corners);
            this->mesh = std::move(t.mesh);
            this->mass = t.mass;
        };
        void transform(_T);
        std::vector<Vector4d> &get_corners();
        friend std::ostream &operator<<(std::ostream &output,
                                        const Link &link)
        {
            for (auto corner : link.corners)
                output << corner.T();
            return output;
        }
    };

    class Cube : public Link
    {
    private:
        ad_se3 inertia;

    public:
        Cube(Vector3d box, Vector3d rpy = Vector3d{0, 0, 0}, Vector3d xyz = Vector3d{0, 0, 0}, double mass = 1);
    };
}