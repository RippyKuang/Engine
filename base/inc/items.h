#pragma once
#include <geometry.h>
#include <vector>

namespace Engine
{
    class Link
    {
        friend class World;
        friend class Camera;
        friend class Joint_node;
        friend class Joint;

    protected:
        _T init_pose;
        double mass;
        Vector4d center;
        void transform(_T);
        std::vector<Vector4d> corners;

    public:
        Link(std::vector<Vector4d> _corners) : corners(_corners) {};
        Link(std::vector<Vector3d> _corners);
        Link() {};

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
    public:
        Cube(std::vector<Vector4d> _corners) : Link(_corners)
        {
            assert(_corners.size() == 8);
        }
        Cube(std::vector<Vector3d> _corners) : Link(_corners)
        {
            assert(_corners.size() == 8);
        }
        Cube(Vector3d box, Vector3d rpy = Vector3d{0, 0, 0}, Vector3d xyz = Vector3d{0, 0, 0}, double mass = 1);
    };
}