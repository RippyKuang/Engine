#include <camera.h>

namespace Engine
{
    Camera::Camera(Vector3d center, _R _intrinsics) : intrisics(_intrinsics)
    {
        corners.push_back(Vector4d(center, 1));
        init_pose = getTransformMat(EYE(3), center);
    }
    
    std::vector<Point2i> Camera::project(Item &pw)
    {
        std::vector<Point2i> corners;
        std::vector<Vector4d> new_corners = remove_hidden(pw.corners);
        for (auto corner : new_corners)
        {
            if (corner[2] == -1)
            {
                corners.push_back(Point2i{-1, -1});
                continue;
            }

            Vector3d temp = this->intrisics * Vector3d{corner[1] / corner[0], corner[2] / corner[0], 1};
            corners.push_back(Point2i(temp));
        }
        return corners;
    }
}
