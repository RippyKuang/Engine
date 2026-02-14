#pragma once
#include <items.h>

namespace Engine
{
    class Camera 
    {
    private:
       
        Vector4d center;

    public:
        _R intrisics;
         _T init_pose;
        Camera(Vector3d center, _R _intrinsics);
        void project(Link &lk);
        void project_all(std::vector<Vector3d> &, std::vector<Point2i> &);
    };
}